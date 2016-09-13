/**
 * 
 */
package segway;

import java.io.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import lejos.ev3.tools.EV3Console;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Key;
import lejos.hardware.Sound;
import lejos.hardware.Sounds;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.BasicMotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.EncoderMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import segway.EV3Gyro;

/**
 * @author José Emilio Traver
 *
 */
public class Stabilizer {
	
	/*
	 * Constantes
	 */
	private final float FALLING_DOWN = (float) (30f); // Umbral de inclinación a partir de cual no se ejecuta el controlador. 		
	public static final float dt = 5f; 	// Tiempo de muestreo (ms) // En lego dt = ( 22 - 2) / 1000

	
	//=====================================================================
	// Variables del Giroscopio
	//=====================================================================	
	private final int sample_filter_raw = 5;
	private final int sample_calibration = 500;
	private final double max_diff_calibration = 0.5;
	
	private double angle_rate_offset = 0;
	
	//=====================================================================
	// Parámetros y Variables de los Motores
	//=====================================================================		
	public static final double DIAMETER_WHEEL = 56; 					// Diametro de las ruedas (mm). 
	public static final double RADIO_WHEEL = DIAMETER_WHEEL/2000;		// Radio de las ruedas (m)

	private long positionwheels = 0;
	private long speedwheels = 0;
	private long positionwheels_diff = 0;
	private long last_positionwheel = 0;
	
	//=====================================================================
	// Constantes controlador PID de estabilidad 
	//=====================================================================	
	private final double kp = 0.5;		// Ganancia proporcional
	private final double ki = 11;		// Ganancia integral
	private final double kd = 0.005;	// Ganancia derivativa
	private final double ktau = 0;		// Ganancia Anti-Windup
	
	//=====================================================================
	// Ponderación LQR de las variables del sistema (Variables Lego)
	//=====================================================================	
	private final double Kpsidot = 1.3; // Ganancia de velocidad angular. 
	private final double Kpsi = 25;
	private final double Kphidot = 75;
	private final double Kphi  = 350; 
	
	
	/**
	 * Indica el estado del robot. 
	 * @true El robot se encuentra desactivado (se ha caído).
	 * @false El robot se encuentra activado (está estabilizado).
	 */
	private boolean stateStabilizer = false;	
	
		
	// Variables del sistema
	private double Psi = 0f;				// Variables compartidas
	private double PsiDot = 0f;			// Variables compartidas
	private double Phi = 0f;				// Variables compartidas
	private double PhiDot = 0f;			// Variables compartidas
	private double steering_sync = 0f;

	private double speed = 0f;			// Variables compartida
	private double old_steering = 0f;
	private double steering = 0f;		// Variables compartida

	
	// Definición de controladores
	
	// Definición de sensores y actuadores
	private EV3GyroSensor gyro;
	private EncoderMotor leftMotor;
	private EncoderMotor rightMotor;
	
	// Mutex - Lock
	private Lock lock_drivecontrol;
	private Lock lock_stabilizer;
	
	/**
	 * Datalloger
	 * Indicar que dato se guardarán
	 */
	PrintWriter stabilizerlog = null;
	
	/* Revisar todo de aqui en adelante */
	private TextLCD lcd;
	private EV3 chip;
	
	/*------------------------
	 * Métodos 
	 -------------------------*/
	
	 /**
	 * Constructor por defecto. 
	 */
	public Stabilizer() {
		super();
	}
	
	 /**
	 * Constructor
	 */
	public Stabilizer(Port PortGyro, Port portleftMotor, Port portrightMotor) {
		
		/* Inicialización Giroscopio */
		Button.LEDPattern(4);
		if(Segway.SOUND)
			Sound.playTone(440, 100, 10);
		
		gyro = new EV3GyroSensor(PortGyro);
		
		// Se calcula el valor de offset para la calibración del giroscopio.
		calibrateGyro();
		
		//filtergyro = new FourthOrderFilter(FourthOrderFilter.CUTOFF_12);
		

		/* Inicialización Motores */
		Button.LEDPattern(5);
		if(Segway.SOUND)
			Sound.playTone(440, 100, 10);
		
		leftMotor = new UnregulatedMotor(portleftMotor,BasicMotorPort.PWM_FLOAT);
		rightMotor = new UnregulatedMotor(portrightMotor,BasicMotorPort.PWM_FLOAT);
		
		// Inicialización encoder de posición. Se toma la posición de inicio como 
		// referencia.
		stopMotor();
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		
		
		/* Inicialización de objetos */
		
		
		/* Atributos de permiso y acceso */
		lock_drivecontrol = new ReentrantLock();
		lock_stabilizer = new ReentrantLock();
		
		/**
		 * Datalogg
		 */
		if (Segway.STABILIZERLOG) {
			try {
				stabilizerlog = new PrintWriter("dataStabilizer.txt", "UTF-8");
				stabilizerlog.println("PsiDot,Psi,Phi,PhiDot,error,integrated_error,derivate_error,control_action,power_rightmotor,power_leftmotor,dt");
			}
			catch (FileNotFoundException e1) {e1.printStackTrace();}
			catch (UnsupportedEncodingException e1) {e1.printStackTrace();}
		}
		
		Button.LEDPattern(0);
		// Indica que se ha conseguido la calibración
		// Play tone: frequency 440Hz, volume 10
		// duration 0.1sec, play type 0
		if(Segway.SOUND)
			Sound.playTone(440, 200, 10);
		
		setStateStabilizer(true);
		
	}	
	
	/**
	 * Determina el valor de offset del Giroscopio. Realiza la media aritmética del número de medidas 
	 * indicadas por sample_calibration.
	 * @param  	none
	 * @return  none
	 */
	private void calibrateGyro(){
				
		// Método de calibración descrito por Lego®.
		// ver http://www.us.lego.com/en-us/mindstorms/community/robot?projectid=96894a3a-45db-48f9-9544-abf66f481b32
		gyro.setCurrentMode("Rate");
		gyro.setCurrentMode("Angle");
		
		try { Thread.sleep(200);} catch (InterruptedException e) {e.printStackTrace();}
		gyro.setCurrentMode("Rate");
		try { Thread.sleep(3300);} catch (InterruptedException e) {e.printStackTrace();}
		
		while(!(getRawGyro() >= 0 || getRawGyro() < 0))
			try { Thread.sleep(200);} catch (InterruptedException e) {e.printStackTrace();}
		
					
		// Se inicializa el giroscopio. Se calcula su valor de offset.
		do{
		
		angle_rate_offset = 0f;	
		
		for (int n = 0; n < sample_calibration; n++ ){
			angle_rate_offset +=getRawGyro();
			try { Thread.sleep(5);} catch (InterruptedException e) {e.printStackTrace();}
			
		}
		
		angle_rate_offset /= sample_calibration;
		
		if (Math.abs( (float) ( getRawGyro() - angle_rate_offset ) ) >= max_diff_calibration){
			// Indicar que hay que mantener quieto al robot
			// Poner cara de gruñon.
	
			System.out.println("No te muevas");
			Button.LEDPattern(5);
		}

		/**
		 *  DEBUG
		 */
		if (Segway.GYRODB)
			System.out.println("Gyro offset: "+ angle_rate_offset);
		
		
		} while(Math.abs( (float) ( getRawGyro() - angle_rate_offset) ) >= max_diff_calibration);	
		
	
	}
	
	
	/**
	 * Devuelve la velocidad de giro medida por el giroscópico sin ningún tipo de tratamiento.
	 * @param  	none
	 * @return  none
	 */
	private double getRawGyro(){
		
		float[] raw_gyro = new float[1];
		double _filter_raw_gyro = 0;
		
		for (int i = 0; i < sample_filter_raw; i++){
			gyro.getRateMode().fetchSample(raw_gyro, 0);
			_filter_raw_gyro += (double) raw_gyro[0];
		}
		
		_filter_raw_gyro /= sample_filter_raw;
		
		/**
		 *  DEBUG
		 */
		if (Segway.GYRODB){
			System.out.println("GYRO: "+ _filter_raw_gyro );
		}
		
		return (float) -_filter_raw_gyro;
	}
	
   /**
    * stop both motors from rotating
    */
   private void stopMotor()
   {
      leftMotor.stop();
      rightMotor.stop();
   }
   
   private void updateMotorData(){
	   
	   long _position_leftmotor,_position_rightmotor, _position;
	   
	   _position_leftmotor = leftMotor.getTachoCount();
	   _position_rightmotor = rightMotor.getTachoCount();
	      
		// Maintain previous mrcSum so that delta can be calculated and get
		// new mrcSum and Diff values
		last_positionwheel = _position;
		_position = (_position_leftmotor + _position_rightmotor) * ( (float) Math.toRadians(1) * RADIO_WHEEL / 2f);;
		motorDiff = _position_leftmotor - _position_rightmotor;

		// mrcDetla is the change int sum of the motor encoders, update
		// motorPos based on this detla
		mrcDelta = mrcSum - mrcSumPrev;
		motorPos += mrcDelta;

		// motorSpeed is based on the average of the last four delta's.
		motorSpeed = (mrcDelta+mrcDeltaP1+mrcDeltaP2+mrcDeltaP3)/(4*tInterval);

		// Shift the latest mrcDelta into the previous three saved delta values
		mrcDeltaP3 = mrcDeltaP2;
		mrcDeltaP2 = mrcDeltaP1;
		mrcDeltaP1 = mrcDelta;
	   
	   
	   
   }
	
	/*
	 * Actualiza el valor de las variables de estado del sistema.
	 */
	private void updateVariableState(){
		
		lock_stabilizer.lock();
		
        // Actualización variables del sistema
		PsiDot = getRawGyro();
        Psi = angle;
        
        // ctrl.tiltAngle() is used to drive the robot forwards and backwards
        Phi =  positionwheels;
        PhiDot = speedwheels;
        
        lock_stabilizer.unlock();
        
		/**
		 * Datalogg
		 */
		if (Segway.STABILIZERLOG) 
			stabilizerlog.print(PsiDot+","+Psi+","+Phi+","+PhiDot);
		
        
        
	}
	
	/*
	 * Devuelve la velocidad de avance de las ruedas.
	 */
	public float getStabilizerSpeed(){
		
		float PhiDot = 0f;
		
		lock_stabilizer.lock();
        PhiDot = this.PhiDot;
        lock_stabilizer.unlock();
        
        return PhiDot;
        
	}

	
	/*
	 * Devuelve el ángulo de giro de las ruedas.
	 */
	public float getStabilizerPosition(){
		
		float Phi = 0f;
		
		lock_stabilizer.lock();
        Phi = this.Phi;
        lock_stabilizer.unlock();
        
        return Phi;
        
	}

	/*
	 * Devuelve el valor de la velocidad de inclinación del robot.
	 */
	public float getStabilizerRateAngle(){
		
		float PsiDot = 0f;
		
		lock_stabilizer.lock();
        PsiDot = this.PsiDot;
        lock_stabilizer.unlock();
        
        return PsiDot;
        
	}
	
	/*
	 * Devuelve el valor del angulo de inclinación del robot.
	 */
	public float getStabilizerAngle(){
		
		float Psi = 0f;
		
		lock_stabilizer.lock();
        Psi = this.Psi;
        lock_stabilizer.unlock();
        
        return Psi;
        
	}
	
	/*
	 * Se pondera el valor de las variables de estado del sistema a partir 
	 * de los pesos proprocionado por el algoritmo de optimización LQR.
	 */
	private void updateWeighingLQR(){
		
		float _speed = getSpeed();
		// refpos += getSpeed();
		refpos = refpos + (dt/1000 * _speed * 0.002f); // Lego
		
		// No es necesario negar los valores de Psi y PsiDot, debido a la ubicación física
		// del sensor que proporciona las variable medida del signo contrario a la referencia.
		error =
				Kpsi * Psi +
				Kpsidot * PsiDot +
				Kphi *  ( refpos - Phi ) +
				Kphidot * (_speed- PhiDot );
		
	}
	
	private float updateController(float error){
		
		float derivate_error =  0f;
		float control_action = 0f;

		
		/* Acción integral */
		// Error integral
		integrated_error += error * (dt/1000);
				
		derivate_error = (error - past_error) / (dt/1000);
		past_error = error;
		
	//	derivate_error = last_dTerm + 0.556864f  * (derivate_error - last_dTerm);
		
	//	last_dTerm = derivate_error;
		
		control_action = error * kp + integrated_error * ki	+ derivate_error * kd;
		
		/* Control anti-Windup */
		if (control_action > EV3Motor.MAX_POWER){
			integrated_error += (EV3Motor.MAX_POWER - control_action) * ktau;
			control_action = EV3Motor.MAX_POWER;
		} else if (control_action < EV3Motor.MIN_POWER){
			integrated_error += (EV3Motor.MIN_POWER - control_action) * ktau;
			control_action = EV3Motor.MIN_POWER;
		}
		
		if (Segway.STABILIZERLOG) 
			stabilizerlog.print(","+error+","+integrated_error+","+derivate_error+","+control_action);
		
		return  control_action;
	}
	
	private void resetController(){
	
		integrated_error = 0f;
		past_error = 0f;		
		last_dTerm = 0f;
				
		return;
	}
	
	public boolean getStateStabilizer(){
		
		boolean stateStabilizer;
		
		lock_stabilizer.lock();
		stateStabilizer = this.stateStabilizer;
		lock_stabilizer.unlock();
		
		return stateStabilizer;
	}
	
	public void setStateStabilizer(boolean state){
		
		lock_stabilizer.lock();
		
		stateStabilizer = state;
		
		lock_stabilizer.unlock();
	}
	
	public void setSteering(float steering){
		
		lock_drivecontrol.lock();
		this.steering = steering;
		lock_drivecontrol.unlock();
	}
	
	public void setSpeed(float speed){
		
		lock_drivecontrol.lock();
		this.speed = speed;
		lock_drivecontrol.unlock();
	}
	
	private float getSpeed(){
		
		float _speed = 0f;
		
		lock_drivecontrol.lock();
		_speed = speed;
		lock_drivecontrol.unlock();
		
		return _speed;
	}
	
	
	private float getSteering(){
		// Posible lock
		// limit steering: [-50, 50]
		float new_steering = 0f;
		
		// Se actualiza el valor de la variable global de la clase a la local del método.
		lock_drivecontrol.lock();
		new_steering = this.steering;
		lock_drivecontrol.unlock();
		
		if (new_steering == 0){
		
			if (old_steering != 0)
					steering_sync = motors.getRightAngle() - motors.getLeftAngle();
			new_steering = (motors.getRightAngle() - motors.getLeftAngle() - steering_sync) * 0.05f;
			return new_steering;
			
		}
		else {
			old_steering = new_steering;
			steering = (new_steering > 50)?50:new_steering;
			steering = (new_steering < -50)?-50:new_steering;
			return -steering / 2;
		}
 

	}
	
	private void logClose(){
		stabilizerlog.flush();
		stabilizerlog.close();
	}
	
	/**
	 * 
	 * Definición del hilo mediante interface de Runnable
	 *
	 */
	private class StabilizerThread implements Runnable { 
	
		private StabilizerThread() {
			super(); 
		}
		
		@Override
		public void run () {
	
			//=====================================================================
			// Variables controlador PID
			//=====================================================================
			double integrated_error = 0;
			double past_error = 0;
			double last_dTerm = 0;
			double error = 0;
			double refpos = 0;
			
			
			long stabilizerTime = 0;
			float power_motors = 0;
			float turns_power_motors = 0;
			int power_rightmotor = 0;
			int power_leftmotor = 0;
			long delay = 0;
			
			
			/*
			 * Loop 100 Hz (Tiempo comprobado entre 8 - 17 ms)
			 */
			while(getStateStabilizer()) {
				// Código a ejecutar de forma concurrente
				// Determinar condicion para salir del bucle cuando el robot se cae
			
				stabilizerTime = System.currentTimeMillis();
				
				//ctrl.setUpright(true);
	            // runDriveState();
	            
				// Actualización variables del sistema.
				updateVariableState();
				
					// Ponderación de las variables de estado.
					updateWeighingLQR();
					
					// Controlador
					// refspeed += getSpeed() * (dt/1000) * 0.002
					// motorcontroller.setPIDParam(PIDController.PID_SETPOINT, refspeed);
	
					power_motors = updateController(error);
					turns_power_motors = getSteering();
					power_rightmotor = (int) ((power_motors - turns_power_motors) * (0.021f / EV3Motor.RADIO_WHEEL));
					power_leftmotor = (int) ((power_motors + turns_power_motors) * (0.021f / EV3Motor.RADIO_WHEEL));
									
					
					if(Math.abs(Psi) < FALLING_DOWN && getStateStabilizer()){
			        	motors.setPower(power_leftmotor,power_rightmotor);
					//	System.out.println(power_motors);
			        //motors.setPower(power_motors, power_motors);
					}
					else {
						motors.stop();
						setStateStabilizer(false);
						if  (Math.abs(Psi) > FALLING_DOWN && !getStateStabilizer()){
							// Se enfada al caerse
							Button.LEDPattern(8);
						}
						if (Button.UP.isDown()){
							setStateStabilizer(true);
							gyro.reset();
							motors.resetMotors();
							resetController();
						}
					}
					
				}
					

				// Se añade 2 ms más para garantizar el tiempo del bucle.
				delay = ( System.currentTimeMillis()-stabilizerTime ) ;
				
				if (Segway.STABILIZERLOG) 
					stabilizerlog.println(","+power_rightmotor+","+power_leftmotor+","+delay);
			
				
				// Revisar delay2 eliminar
				long delay2 = 0;
				if (delay >= dt)
					delay2 = (int) dt;
				else
					delay2 = (int) (dt - delay);
					            
	            // Delay used to stop Gyro being read to quickly. May need to be increase or
	            // decreased depending on leJOS version.
				try {Thread.sleep(delay2);} catch (Exception e) {}
									
			}
			
			// Se cierra la comunicación con los sensores y actuadores.
			motors.stop();
			//gyro.close();
			
			if (Segway.GYROLOG) gyro.logClose();
			if (Segway.MOTORLOG) motors.logClose();
			if (Segway.STABILIZERLOG) logClose();
			
		}
	}
	
	/**
     * Invoca e inicia el Thread Stabilizer para estabilizar el robot o
	 * 			o lograr su desplazamiento sin perder el equilibrio.
	 * @param - none
	 * @return 	none
	 */
	public void start() {
		Thread segwaythread = new Thread(new StabilizerThread());
		segwaythread.setPriority(Thread.MAX_PRIORITY);
		segwaythread.start();
		
	}

}
