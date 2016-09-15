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
	
	/**
	 *  Umbral de inclinación a partir de cual no se ejecuta el controlador. 
	 */
	private final int FALLING_DOWN = 30; 
	
	/** 
	 * If robot power is saturated (over +/- 100) for over this time limit then 
	 * robot must have fallen.  In milliseconds.
	 */
	private static final double TIME_FALL_LIMIT = 1000; // originally 1000
	
	public static final int dt = 5; 	// Tiempo de muestreo (ms) // En lego dt = ( 22 - 2) / 1000

	
	//=====================================================================
	// Variables del Giroscopio
	//=====================================================================	
	private final int sample_filter_raw = 5;
	private final int sample_calibration = 500;
	private final double max_diff_calibration = 0.5;
	
	private double angle = 0;
	private double angle_rate_offset = 0;
	private double filter_angle_rate = 0;
	
	//=====================================================================
	// Parámetros y Variables de los Motores
	//=====================================================================		
	public static final double DIAMETER_WHEEL = 56; 					// Diametro de las ruedas (mm). 
	public static final double RADIO_WHEEL = DIAMETER_WHEEL/2000;		// Radio de las ruedas (m)

	private double positionwheels = 0;
	private double speedwheels = 0;
	private double positionwheels_diff = 0;
	private double last_positionwheel = 0;
	private double positiondelta1 = 0;
	private double positiondelta2 = 0;
	private double positiondelta3 = 0;
	
	//=====================================================================
	// Constantes controlador PID de estabilidad 
	//=====================================================================	
	private final double kp = 0.5;		// Ganancia proporcional
	private final double ki = 11;		// Ganancia integral
	private final double kd = 0.0005;	// Ganancia derivativa
	
	private double ref_speed = 0;
	
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
	private double Psi = 0;
	private double PsiDot = 0;
	private double Phi = 0;
	private double PhiDot = 0;
	private double steering = 0;
	private double new_steering = 0;
	private double steering_sync = 0;
	
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
		gyro.setCurrentMode(2);	
	/*	try { Thread.sleep(200);} catch (InterruptedException e) {e.printStackTrace();}
		gyro.setCurrentMode("Rate");
		try { Thread.sleep(3300);} catch (InterruptedException e) {e.printStackTrace();}
	*/	while(!(angle >= 0 || angle < 0))
			try { Thread.sleep(200);} catch (InterruptedException e) {e.printStackTrace();}
		
				
	/*	// Se inicializa el giroscopio. Se calcula su valor de offset.
		do{
		
		angle_rate_offset = 0;	
		
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
		 *
		if (Segway.GYRODB)
			System.out.println("Gyro offset: "+ angle_rate_offset);
		
		
		} while(Math.abs( (float) ( getRawGyro() - angle_rate_offset) ) >= max_diff_calibration);	
		
	*/	
	}
	
	/**
	 * Devuelve la velocidad de giro medida por el giroscópico sin ningún tipo de tratamiento.
	 * @param  	none
	 * @return  none
	 */
	private double getRawGyro(){
		
		float[] raw_gyro = new float[2];
		double _filter_raw_gyro = 0;
		double _angle_rate = 0;

		
//		for (int i = 0; i < sample_filter_raw; i++){
			gyro.getAngleAndRateMode().fetchSample(raw_gyro, 0);
		//	gyro.getRateMode().fetchSample(raw_gyro, 0);
//			_filter_raw_gyro -= (double) raw_gyro[0];
//		}
		
		//_filter_raw_gyro /= sample_filter_raw;
		
		// EMA
	//	filter_angle_rate = filter_angle_rate * (1 - 0.2 * Stabilizer.dt/1000) + ((_filter_raw_gyro-angle_rate_offset) * 0.2 * Stabilizer.dt/1000);
	//	_angle_rate  = (_filter_raw_gyro-angle_rate_offset) - filter_angle_rate;
		
	//	angle = angle +  _angle_rate * (double) (Stabilizer.dt)/1000;
	
		angle = raw_gyro[0];
		/**
		 * Datalog
		 */
	//	if (Segway.GYROLOG) 
	//		gyrolog.print(angle_rate_offset+","+_angle_rate+","+ (getRawGyro()-angle_rate_offset) );
		
		
		/**
		 *  DEBUG
		 */
		if (Segway.GYRODB){
			System.out.println("GYRO: "+ _filter_raw_gyro );
		}
		
		//return _angle_rate;
		return raw_gyro[1];
	}
	
	/**
	 * Calcula en ángulo de inclinación y devuelve la velocidad de giro.
	 * @return _angle_rate (rad/s). Velocidad de giro.
	 */
	private double getRateAngle(){
		
		double _angle_rate =0;
				
		// EMA
		filter_angle_rate = filter_angle_rate * (1 - 0.2 * Stabilizer.dt/1000) + ((getRawGyro()-angle_rate_offset) * 0.2 * Stabilizer.dt/1000);
		_angle_rate  = (getRawGyro()-angle_rate_offset) - filter_angle_rate;
		
		angle = angle +  _angle_rate * (double) (Stabilizer.dt)/1000;
		
		/**
		 * Datalog
		 */
	//	if (Segway.GYROLOG) 
	//		gyrolog.print(angle_rate_offset+","+_angle_rate+","+ (getRawGyro()-angle_rate_offset) );
		
		return _angle_rate;
	}
	
   /**
    * stop both motors from rotating
    */
   private void stopMotor()
   {
      leftMotor.stop();
      rightMotor.stop();
   }
   
   /**
    * Calcula la velocidad de las ruedas y devuelve el desplazamiento.   
    * @return positionwheels (m). Desplazamiento de las ruedas.
    */
   private double getPositionWheel(){
	   
	   double _position_leftmotor,_position_rightmotor, _position = 0, _positiondelta ;
	   
	   _position_leftmotor = leftMotor.getTachoCount();
	   _position_rightmotor = rightMotor.getTachoCount();
	      
		// Maintain previous mrcSum so that delta can be calculated and get
		// new mrcSum and Diff values
		_position = _position_leftmotor + _position_rightmotor;
		positionwheels_diff = _position_leftmotor - _position_rightmotor;

		// mrcDetla is the change int sum of the motor encoders, update
		// motorPos based on this detla
		_positiondelta = _position - last_positionwheel;
		positionwheels += _positiondelta * ( Math.toRadians(1) * RADIO_WHEEL / 2f);
		last_positionwheel = _position;
		
		// motorSpeed is based on the average of the last four delta's.
		speedwheels = (long) (RADIO_WHEEL * (_positiondelta+positiondelta1+positiondelta2+positiondelta3)/(4*Stabilizer.dt/1000));

		// Shift the latest mrcDelta into the previous three saved delta values
		positiondelta3 = positiondelta2;
		positiondelta2 = positiondelta1;
		positiondelta1 = _positiondelta;
		
		return (double) positionwheels;
	   
   }
	
	/*
	 * Actualiza el valor de las variables de estado del sistema.
	 */
	private void updateVariableState(){
		
		lock_stabilizer.lock();
		
        // Actualización variables del sistema
		PsiDot = getRawGyro();
        Psi = angle;
        
        Phi =  getPositionWheel();
        PhiDot = (double) speedwheels;
        
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
	public double getStabilizerSpeed(){
		
		double PhiDot = 0;
		
		lock_stabilizer.lock();
        PhiDot = this.PhiDot;
        lock_stabilizer.unlock();
        
        return PhiDot;
        
	}

	
	/*
	 * Devuelve el ángulo de giro de las ruedas.
	 */
	public double getStabilizerPosition(){
		
		double Phi = 0;
		
		lock_stabilizer.lock();
        Phi = this.Phi;
        lock_stabilizer.unlock();
        
        return Phi;
        
	}

	/*
	 * Devuelve el valor de la velocidad de inclinación del robot.
	 */
	public double getStabilizerRateAngle(){
		
		double PsiDot = 0;
		
		lock_stabilizer.lock();
        PsiDot = this.PsiDot;
        lock_stabilizer.unlock();
        
        return PsiDot;
        
	}
	
	/*
	 * Devuelve el valor del angulo de inclinación del robot.
	 */
	public double getStabilizerAngle(){
		
		double Psi = 0;
		
		lock_stabilizer.lock();
        Psi = this.Psi;
        lock_stabilizer.unlock();
        
        return Psi;
        
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
		new_steering = steering;
		lock_drivecontrol.unlock();
	}
	
	private double getSpeed(){
		
		double _speed = 0f;
		
		lock_drivecontrol.lock();
		_speed = ref_speed;
		lock_drivecontrol.unlock();
		
		return _speed;
	}
	
	private double getSteering(){
		// Posible lock
		// limit steering: [-50, 50]
		double new_steering = 0;
		
		// Se actualiza el valor de la variable global de la clase a la local del método.
		lock_drivecontrol.lock();
		new_steering = this.new_steering;
		lock_drivecontrol.unlock();
		
		if (new_steering == 0){
		
			if (steering != 0)
					steering_sync = positionwheels_diff;
			new_steering = (positionwheels_diff - steering_sync) * 0.05f;
			return new_steering;
			
		}
		else {
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
			
			long time_motorspeedOK = 0;
			double ref_position = 0;
			double speed = 0;
			
			//=====================================================================
			// Variables controlador PID
			//=====================================================================
			double integrated_error = 0;
			double past_error = 0;
			double last_dTerm = 0;
			double error = 0;
			
			double derivate_error =  0;
			
			int count_scheduler = 0;	// Variable de planificación
			long stabilizerTime = 0;
			double power_motors = 0;
			double turns_power_motors = 0;
			int power_rightmotor = 0;
			int power_leftmotor = 0;
			
			
			/*
			 * Loop 100 Hz (Tiempo comprobado entre XX - XX ms)
			 */
			while(getStateStabilizer()) {
				
				stabilizerTime = System.currentTimeMillis();
			
				//ctrl.setUpright(true);
	            // runDriveState();
	            
				// Actualización variables del sistema.
				updateVariableState();
				
				/*
				 * Loop 50 Hz (Tiempo comprobado entre XX - XX ms)
				 */
		//		if ((count_scheduler % LOOP_50Hz) == 0){
				
				/*
				 * Se pondera el valor de las variables de estado del sistema a partir 
				 * de los pesos proprocionado por el algoritmo de optimización LQR.
				 */
								
				ref_position += (dt/1000 * getSpeed() * 0.002f); // Lego
				
				// No es necesario negar los valores de Psi y PsiDot, debido a la ubicación física
				// del sensor que proporciona las variable medida del signo contrario a la referencia.
				error =	Kpsi * Psi +
						Kpsidot * PsiDot +
						Kphi *  ( ref_position - Phi ) +
						Kphidot * (getSpeed() - PhiDot );
					
				/* Controlador */
				integrated_error += error * (double) (dt/1000);		
				if (past_error != 0)
					derivate_error = (error - past_error) / (double) (dt/1000);
				past_error = error;			
				//derivate_error = last_dTerm + 0.556864f  * (derivate_error - last_dTerm);
				//last_dTerm = derivate_error;
				power_motors = error * kp + integrated_error * ki	+ derivate_error * kd;
				turns_power_motors = getSteering();
				//turns_power_motors = getSteering();
				
				if (Segway.STABILIZERLOG) 
					stabilizerlog.println(","+power_rightmotor+","+power_leftmotor);
				
				power_rightmotor =  (int) ((power_motors + turns_power_motors) * (0.021f / EV3Motor.RADIO_WHEEL));
				power_leftmotor =  (int) ((power_motors - turns_power_motors) * (0.021f / EV3Motor.RADIO_WHEEL));
				
				// Limit the power to motor power range -100 to 100
				if (power_rightmotor > 100)   power_rightmotor = 100;
				if (power_rightmotor < -100)  power_rightmotor = -100;

				// Limit the power to motor power range -100 to 100
				if (power_leftmotor > 100)  power_leftmotor = 100;
				if (power_leftmotor < -100) power_leftmotor = -100;
				
				if (Math.abs(power_rightmotor) < 100)
					time_motorspeedOK = stabilizerTime;
					
				/* Potencia motores */
				leftMotor.setPower(Math.abs(power_leftmotor));
				rightMotor.setPower(Math.abs(power_rightmotor));
				
				if (power_leftmotor < 0) leftMotor.backward();
				else leftMotor.forward();
		       
				if (power_rightmotor < 0)  rightMotor.backward(); 
				else rightMotor.forward();

				//System.out.println((double)(System.currentTimeMillis() - stabilizerTime));
				System.out.println("G: "+ PhiDot+"A: "+ Phi);
				
				
				// Check if robot has fallen by detecting that motorPos is being limited
				// for an extended amount of time.
			//	if ((System.currentTimeMillis() - time_motorspeedOK) > TIME_FALL_LIMIT || Math.abs(Psi) > FALLING_DOWN) break;

				
	            // Delay used to stop Gyro being read to quickly. May need to be increase or
	            // decreased depending on leJOS version.
				try {Thread.sleep(Stabilizer.dt);} catch (Exception e) {}
			}
			
			System.out.println(PsiDot);

			leftMotor.flt();
			rightMotor.flt();
			setStateStabilizer(false);
			
			// Se enfada al caerse
			Button.LEDPattern(8);
	
			
		}
	/*		
			if (Segway.GYROLOG) gyro.logClose();
			if (Segway.MOTORLOG) motors.logClose();
			if (Segway.STABILIZERLOG) logClose();
	*/		
			// Se cierra la comunicación con los sensores y actuadores.
			//gyro.close();
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