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
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.Image;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.BasicMotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.port.UARTPort;
import lejos.hardware.sensor.BaseSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.internal.ev3.EV3GraphicsLCD;
import lejos.robotics.EncoderMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import segway.EV3Gyro;
import segway.GyroEV3;


/**
 * @author José Emilio Traver
 *
 */
public class Stabilizer {
	

	/**
	 *  Umbral de inclinación a partir de cual no se ejecuta el controlador. 
	 */
	private final int FALLING_DOWN = 30;
	private static final int LOOP_50Hz = 4;
	private static final int SLOWEST_LOOP = 4*LOOP_50Hz;

	
	/** 
	 * If robot power is saturated (over +/- 100) for over this time limit then 
	 * robot must have fallen.  In milliseconds.
	 */
	private static final double TIME_FALL_LIMIT = 500; // originally 1000
	
	public static final int dt = 20; 	// Tiempo de muestreo (ms) // En lego dt = ( 22 - 2) / 1000

	
	//=====================================================================
	// Variables del Giroscopio
	//=====================================================================	
	private final int SAMPLE_FILTER_RAW = 5;
	private final int SAMPLE_CALIBRATION = 500;
	private final float MAX_DIFF_CALIBRATION = 1f;
	
	private float angle = 0f;
	private float angle_rate_offset = 0f;
	private float filter_angle_rate = 0f;
	
	//=====================================================================
	// Parámetros y Variables de los Motores
	//=====================================================================		
	public static final float DIAMETER_WHEEL = 56; 					// Diametro de las ruedas (mm). 
	public static final float RADIO_WHEEL = DIAMETER_WHEEL/2000;		// Radio de las ruedas (m)
	
	private float positionwheels = 0f;
	private float speedwheels = 0f;
	private float positionwheels_diff = 0f;
	private float last_positionwheel = 0f;
	
	//=====================================================================
	// Constantes controlador PID de estabilidad 
	//=====================================================================	
	private final float kp = 0.5f;		// Ganancia proporcional
	private final float ki = 11f;		// Ganancia integral
	private final float kd = 0.005f;	// Ganancia derivativa
	
	private float ref_position = 0f;
	private static float ref_speed = 0f;
	
	/* Consignas */
	private float integrated_error = 0f;
	private float past_error = 0f;
	private float last_dTerm = 0f;
	
	//=====================================================================
	// Ponderación LQR de las variables del sistema (Variables Lego)
	//=====================================================================	
	private final float Kpsidot = 1.3f; // Ganancia de velocidad angular. 
	private final float Kpsi = 25f;		//25
	private final float Kphidot = 100f; // 75
	private final float Kphi  = 315f; 	// 350
	
		
	//=====================================================================
	// Variables del sistema
	//=====================================================================	
	private float Psi = 0f;
	private float PsiDot = 0f;
	private float Phi = 0f;
	private float PhiDot = 0f;
	private float steering = 0f;
	private static float new_steering = 0f;
	private float steering_sync = 0f;
	
	/**
	 * Indica el estado del robot. 
	 * @true El robot se encuentra desactivado (se ha caído).
	 * @false El robot se encuentra activado (está estabilizado).
	 */
	private boolean stateStabilizer = false;	
	

	// Definición de sensores y actuadores
	private GyroEV3 gyro;
	private EncoderMotor leftMotor;
	private EncoderMotor rightMotor;
	
	// Mutex - Lock
	private static Lock lock_drivecontrol;
	private static Lock lock_stabilizer;
	
	/**
	 * Datalloger
	 * Indicar que dato se guardarán
	 */
	PrintWriter stabilizerlog = null;
	
		
	//============
	// Métodos
	//===========
	
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
		
		/* Inicialización de atributos y declaración de objetos */
	
		// Inicialización Giroscopio
		if (Segway.LED)
			Button.LEDPattern(6);
		if (Segway.LCD)
			LegoImage.displayLegoImage("Crazy 2.rgf",EV3GraphicsLCD.TRANS_NONE);
		
		if(Segway.SOUND)
			Sound.playTone(440, 100, 10);
		
		gyro = new GyroEV3(PortGyro);
		
		// Se calcula el valor de offset para la calibración del giroscopio.
		calibrateGyro();
		
		// Inicialización Motores
		// Play tone: frequency 440Hz, volume 10
		// duration 0.10sec, play type 0
		if(Segway.SOUND)
			Sound.playTone(440, 100, 10);
		if (Segway.LCD)
			LegoImage.displayLegoImage("Crazy 1.rgf",EV3GraphicsLCD.TRANS_NONE);
		
		leftMotor = new UnregulatedMotor(portleftMotor,BasicMotorPort.PWM_FLOAT);
		rightMotor = new UnregulatedMotor(portrightMotor,BasicMotorPort.PWM_FLOAT);
					
		// Inicialización de los encoder de posición. Se toma la posición de inicio como 
		// referencia.
		stopMotor();
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		
		/**
		 * Datalogg
		 */
		if (Segway.STABILIZERLOG) {
			try {stabilizerlog = new PrintWriter("dataStabilizer.txt", "UTF-8");
			stabilizerlog.println("PsiDot,Psi,Phi,PhiDot,error,integrated_error,derivate_error,control_action,power_rightmotor,power_leftmotor");}
			catch (FileNotFoundException e1) {e1.printStackTrace();}
			catch (UnsupportedEncodingException e1) {e1.printStackTrace();}
		}
		

		/* Atributos de permiso y acceso */
		lock_drivecontrol = new ReentrantLock();
		lock_stabilizer = new ReentrantLock();
		
			
		// Se indica que puede iniciar el control de calibración.
		setStateStabilizer(true);
				
		// Indica que se ha conseguido la calibración
		// Play tone: frequency 840Hz, volume 10
		// duration 0.1sec, play type 0
		if(Segway.SOUND)
			Sound.playTone(840, 300, 10);
		if (Segway.LCD)
			LegoImage.displayLegoImage("Neutral.rgf",EV3GraphicsLCD.TRANS_NONE);
		if (Segway.LED)
			Button.LEDPattern(1);
			
	}
		
	/**
	 * Determina el valor de offset del Giroscopio. Realiza la media aritmética del número de medidas 
	 * indicadas por sample_calibration.
	 * @param  	none
	 * @return  none
	 */
	private void calibrateGyro(){
		
		float _angle_rate_offset = 0;
				
		// Método de calibración descrito por Lego®.
		// ver http://www.us.lego.com/en-us/mindstorms/community/robot?projectid=96894a3a-45db-48f9-9544-abf66f481b32
		gyro.reset();
		gyro.setCurrentMode("Rate");

		while(!(getGyro() >= 0 || getGyro() < 0))
			try { Thread.sleep(10);} catch (InterruptedException e) {e.printStackTrace();}
		
				
		// Se inicializa el giroscopio. Se calcula su valor de offset.
		do{
		
		_angle_rate_offset = 0;	
		
		for (int n = 0; n < SAMPLE_CALIBRATION; n++ ){
			_angle_rate_offset +=getGyro();
			if (Segway.LCD)
				if (n % 2 == 0)
					LegoImage.displayLegoImage("Crazy 1.rgf",EV3GraphicsLCD.TRANS_NONE);
				else
					LegoImage.displayLegoImage("Crazy 2.rgf",EV3GraphicsLCD.TRANS_NONE);
			
			try { Thread.sleep(5);} catch (InterruptedException e) {e.printStackTrace();}
		}
		
		_angle_rate_offset /= SAMPLE_CALIBRATION;
		
		if (Math.abs( (float) ( getGyro() - _angle_rate_offset) ) >= MAX_DIFF_CALIBRATION){
			// Indicar que hay que mantener quieto al robot
			// Poner cara de gruñon.
	
			if (Segway.LCD)
				LegoImage.displayLegoImage("Evil.rgf",EV3GraphicsLCD.TRANS_NONE);
			// Play tone: frequency 240Hz, volume 10
			// duration 0.30sec, play type 0
			if(Segway.SOUND)
				Sound.playTone(140, 500, 10);
			
			Button.LEDPattern(5);
		}
		/**
		 *  DEBUG
		 */
		if (Segway.GYRODB)
			System.out.println("Gyro offset: "+ angle_rate_offset);
		
		
		} while(Math.abs( (float) (getGyro() - _angle_rate_offset) ) >= MAX_DIFF_CALIBRATION);	
		
		angle_rate_offset = _angle_rate_offset;	
	}
	
	/**
	 * Devuelve la velocidad de giro medida por el giroscópico sin ningún tipo de tratamiento.
	 * @param  	none
	 * @return  none
	 */
	private float getGyro(){
		
		float[] raw_gyro = new float[1];
		float _filter_raw_gyro = 0;
		float _angle_rate = 0;

		
		for (int i = 0; i < SAMPLE_FILTER_RAW; i++){
			gyro.getRateMode().fetchSample(raw_gyro, 0);
			_filter_raw_gyro += (float) raw_gyro[0];
		}
		
		_filter_raw_gyro /= SAMPLE_FILTER_RAW;
		
		
		// EMA
		filter_angle_rate = filter_angle_rate * (1f - 0.2f * Stabilizer.dt/1000f) + ((_filter_raw_gyro-angle_rate_offset) * 0.2f * Stabilizer.dt/1000f);
		_angle_rate  = (-raw_gyro[0]-angle_rate_offset) - filter_angle_rate;
			
		
		angle = angle +  _angle_rate * (float) (dt)/1000f;
	
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
		
		return _angle_rate;
		//return raw_gyro[1];
	}
	
	
	/**
  	* Stop both motors from rotating
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
	private float getPositionWheel(){
		
		last_positionwheel = positionwheels;
		positionwheels = (leftMotor.getTachoCount() + rightMotor.getTachoCount()) * ( (float) Math.toRadians(1) * RADIO_WHEEL / 2f);
		
		speedwheels = (positionwheels - last_positionwheel) / ((float)dt/1000f);		

		return (float) positionwheels;	
	}

	/*
	 * Actualiza el valor de las variables de estado del sistema.
	 */
	private void updateVariableState(){
		
		lock_stabilizer.lock();
		
        // Actualización variables del sistema
		PsiDot = getGyro();
        Psi = angle;
        
         Phi = getPositionWheel();
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
		
		float PhiDot = 0;
		
		lock_stabilizer.lock();
        PhiDot = this.PhiDot;
        lock_stabilizer.unlock();
        
        return PhiDot;
        
	}

	
	/*
	 * Devuelve el ángulo de giro de las ruedas.
	 */
	public float getStabilizerPosition(){
		
		float Phi = 0;
		
		lock_stabilizer.lock();
        Phi = this.Phi;
        lock_stabilizer.unlock();
        
        return Phi;
        
	}

	/*
	 * Devuelve el valor de la velocidad de inclinación del robot.
	 */
	public float getStabilizerRateAngle(){
		
		float PsiDot = 0;
		
		lock_stabilizer.lock();
        PsiDot = this.PsiDot;
        lock_stabilizer.unlock();
        
        return PsiDot;
        
	}
	
	/*
	 * Devuelve el valor del angulo de inclinación del robot.
	 */
	public float getStabilizerAngle(){
		
		float Psi = 0;
		
		lock_stabilizer.lock();
        Psi = this.Psi;
        lock_stabilizer.unlock();
        
        return Psi;
        
	}
	
	private float updateController(float error){
		
		float derivate_error =  0f;
		float control_action = 0f;

		
		/* Acción integral */
		// Error integral
		integrated_error += error * (dt/1000f) * ki;
	
		if (past_error != 0)
			derivate_error = (error - past_error) / (dt/1000f);
		past_error = error;
		
		derivate_error = last_dTerm + 0.556864f  * (derivate_error - last_dTerm);
		
		last_dTerm = derivate_error;
		
		control_action = error * kp + integrated_error 	+ derivate_error * kd ;
		
		
		if (Segway.STABILIZERLOG) 
			stabilizerlog.print(","+error+","+integrated_error+","+derivate_error+","+control_action);
		
		return  control_action;
	}
	
	private void reset(){
		
		lock_stabilizer.lock();
			angle = 0f;
			positionwheels = 0f;
			speedwheels = 0f;
		lock_stabilizer.unlock();
		
		filter_angle_rate = 0f;
		
		ref_position = 0f;
		positionwheels_diff = 0f;
		last_positionwheel = 0f;
		
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
	
	public static void setSteering(float steering){
		
		lock_drivecontrol.lock();
		new_steering = steering;
		lock_drivecontrol.unlock();
	}
	
	public static float getSteeringController(){
		
		float _new_steering = 0f;
		
		lock_drivecontrol.lock();
		_new_steering = new_steering;
		lock_drivecontrol.unlock();	
		
		return _new_steering;
	}
	
	private float getSteering(){
		// Posible lock
		// limit steering: [-50, 50]
		float new_steering = 0f;
		
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
			steering = (new_steering > 50)?50f:new_steering;
			steering = (new_steering < -50)?-50f:new_steering;
			return -steering / 2;
		}
 

	}
	
	private float getSpeed(){
		
		float _speed = 0f;
		
		lock_drivecontrol.lock();
		_speed = ref_speed;
		lock_drivecontrol.unlock();
		
		return _speed;
	}
	
	public static void setSpeed(float _speed){
				
		lock_drivecontrol.lock();
		ref_speed = _speed;
		lock_drivecontrol.unlock();
		
		return;
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
			
			//==============================
			// Variables controlador PID
			//==============================
			
			long time_motorspeedOK = 0;
			long stabilizerTime = 0;
			
			float error = 0f;
			float power_motors = 0f;
			float turns_power_motors = 0f;
			
			int power_rightmotor = 0;
			int power_leftmotor = 0;
			
			/*
			* Loop 50 Hz (Tiempo comprobado entre XX - XX ms)
			*/
		//	do{
				
				while(getStateStabilizer()) {
					// Código a ejecutar de forma concurrente
					// Determinar condicion para salir del bucle cuando el robot se cae
				
					stabilizerTime = System.currentTimeMillis();
					
		            
					// Actualización variables del sistema.
					updateVariableState();
							
					/*
					 * Se pondera el valor de las variables de estado del sistema a partir 
					 * de los pesos proprocionado por el algoritmo de optimización LQR.
					 */

					ref_position += (getSpeed() * 0.001 * 0.1); // Lego
					
					lock_stabilizer.lock();
						error =	Kpsi * Psi +
								Kpsidot * PsiDot +
								Kphi *  ( Phi - ref_position) +
								Kphidot * (PhiDot);
					lock_stabilizer.unlock();
					
					//================
					//  Controlador 
					//================
			
					power_motors = updateController(error);
					
					turns_power_motors = getSteering();
					power_rightmotor = (int) ((power_motors + turns_power_motors) * (0.021f / EV3Motor.RADIO_WHEEL));
					power_leftmotor = (int) ((power_motors - turns_power_motors) * (0.021f / EV3Motor.RADIO_WHEEL));
					
					if (Segway.STABILIZERLOG) 
						stabilizerlog.println(","+power_rightmotor+","+power_leftmotor);
					
					if (Math.abs(power_rightmotor) < 100 || Segway.MOTORONDB)
						time_motorspeedOK = stabilizerTime;
	
					if (power_rightmotor > 100)   power_rightmotor = 100;
					if (power_rightmotor < -100)  power_rightmotor = -100;
	
					// Limit the power to motor power range -100 to 100
					if (power_leftmotor > 100)  power_leftmotor = 100;
					if (power_leftmotor < -100) power_leftmotor = -100;	
						
					/* Potencia motores */
					
					if (!Segway.MOTORONDB){
					
						if (power_leftmotor < 0){ 
							leftMotor.setPower(-power_leftmotor);
							leftMotor.backward();
						}
						else{
							leftMotor.setPower(power_leftmotor);
							leftMotor.forward();
						}
						
						
						if (power_rightmotor < 0){
							rightMotor.setPower(-power_rightmotor);
							rightMotor.backward(); 
						}
						else{
							rightMotor.setPower(power_rightmotor);
							rightMotor.forward();
						}
					}
					
					
						
					if ((System.currentTimeMillis() - time_motorspeedOK) > TIME_FALL_LIMIT || Math.abs(Psi) > FALLING_DOWN) break;
	
						            
		            // Delay used to stop Gyro being read to quickly. May need to be increase or
		            // decreased depending on leJOS version.
					try {Thread.sleep(dt);} catch (Exception e) {}
										
				}
							
				leftMotor.flt();
				rightMotor.flt();
				setStateStabilizer(false);
				
				if (Segway.LCD)
					LegoImage.displayLegoImage("Black eye.rgf",EV3GraphicsLCD.TRANS_NONE);
				
			/*	if (Button.ENTER.isDown()){
					reset();
					if (Segway.LCD)
						LegoImage.displayLegoImage("Neutral.rgf",EV3GraphicsLCD.TRANS_NONE);
					setStateStabilizer(true);
				}
			*/
			if (Segway.STABILIZERLOG) logClose();
			
			// Se cierra la comunicación con los sensores y actuadores.
			//gyro.close();
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