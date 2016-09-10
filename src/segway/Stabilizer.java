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
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
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
	private static final int LOOP_50Hz = 2;
	private static final int SLOWEST_LOOP = 2*LOOP_50Hz;
	
	private final float FALLING_DOWN = (float) (30f); // Umbral de inclinación a partir de cual no se ejecuta el controlador. 
		
	public static final double dt = 20; 	// Tiempo de muestreo (ms) // En lego dt = ( 22 - 2) / 1000

	
	// Sintonización variables del controlador 	
	private final double kp = 0.5;		// Ganancia proporcional
	private final double ki = 11;			// Ganancia integral
	private final double kd = 0.005;		// Ganancia derivativa
	//private final double kp = 1.2 ; 
	//private final double ki = 0.25;
	//private final double kd = 0.1 ;
	private final double ktau = 0.0005;		// Ganancia Anti
	/* Consignas */
	private double integrated_error = 0;
	private double past_error = 0;
	private double last_dTerm = 0;
	private double error = 0;
	private double refpos = 0;
	
	/**
	 * Indica el estado del robot. 
	 * @true El robot se encuentra desactivado (se ha caído).
	 * @false El robot se encuentra activado (está estabilizado).
	 */
	private boolean stateStabilizer = false;	
	
	// Ponderación de las variables del sistema (Variables Lego)
	private double Kpsidot = 1.3; // Ganancia de velocidad angular. 
	private double Kpsi = 25;
	private double Kphidot = 75;
	private double Kphi  = 350; 
	
	
   // Teseing error contributions.
   //private final double Kpsi = 44.257035; // Gyro angle weight
   //private final double Kphi = 0.806876; // Motor angle weight
   //private final double Kpsidot = 0.620882; // Gyro angle velocity weight
   //private final double Kphidot = 0.039711;// Motor angle velocity weight
   
   // Original Balance numbers
   // private final double Kpsi = 34.189581; // Gyro angle weight
   // private final double Kphi = 0.835082; // Motor angle weight
   // private final double Kpsidot = 0.646772; // Gyro angle velocity weight
   // private final double Kphidot = 0.028141; // Motor angle velocity weight
	
	// Variables del sistema
	private double Psi = 0;
	private double PsiDot = 0;
	private double Phi = 0;
	private double PhiDot = 0;
	private double steering = 0;
	private double new_steering = 0;
	private double steering_sync = 0;
	
	// Definición de controladores
	
	// Definición de sensores y actuadores
	public EV3Gyro gyro;
	private EV3Motor motors;
	
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
	public int delay = 0;
	
	
	 /**
	 * Constructor por defecto. 
	 */
	public Stabilizer() {
		super();
	}
	
	 /**
	 * Constructor
	 */
	public Stabilizer(EV3 device, Port PortGyro, Port portleftMotor, Port portrightMotor) {
		
		/* Inicialización de atributos y declaración de objetos */
		chip = device;
		gyro = new EV3Gyro(device,PortGyro);
		motors = new EV3Motor(portleftMotor, portrightMotor);
		lcd = device.getTextLCD();
		
		/* Inicialización de objetos */
		
		
		/* Atributos de permiso y acceso */
		lock_drivecontrol = new ReentrantLock();
		lock_stabilizer = new ReentrantLock();
		
		/**
		 * Datalogg
		 */
		if (Segway.STABILIZERLOG) {
			try {stabilizerlog = new PrintWriter("dataStabilizer.txt", "UTF-8");
			stabilizerlog.println("PsiDot,Psi,Phi,PhiDot,error,integrated_error,derivate_error,control_action,power_rightmotor,power_leftmotor");}
			catch (FileNotFoundException e1) {e1.printStackTrace();}
			catch (UnsupportedEncodingException e1) {e1.printStackTrace();}
		}
		
		setStateStabilizer(false);
		
	}	
	
	/*
	 * Actualiza el valor de las variables de estado del sistema.
	 */
	private void updateVariableState(){
		
		lock_stabilizer.lock();
		
        // Actualización variables del sistema
		PsiDot = gyro.getRateAngle();
        Psi = gyro.getAngle();
        
        // ctrl.tiltAngle() is used to drive the robot forwards and backwards
        Phi = motors.getPosition();// - ctrl.tiltAngle();
        PhiDot = motors.getSpeed();
        
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
	
	/*
	 * Se pondera el valor de las variables de estado del sistema a partir 
	 * de los pesos proprocionado por el algoritmo de optimización LQR.
	 */
	private void updateWeighingLQR(){
		
		// refpos += getSpeed();
		//refpos = refpos + (dt/1000 * speed * 0.002f); // Lego
		error = Kpsi * Psi +
				Kpsidot * PsiDot +
				Kphi * ( Phi - refpos) +
				Kphidot * PhiDot;
		
	}
	
	private double updateController(double error){
		
		double derivate_error =  0;
		double control_action = 0;

		
		/* Acción integral */
		// Error integral
		integrated_error += error * (dt/1000);
				
		derivate_error = (error - past_error) / (dt/1000);
		past_error = error;
		
		derivate_error = last_dTerm + 0.556864f  * (derivate_error - last_dTerm);
		
		last_dTerm = derivate_error;
		
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
		

		integrated_error = 0;
		past_error = 0;		
		last_dTerm = 0;
				
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
		new_steering = steering;
		lock_drivecontrol.unlock();
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
					steering_sync = motors.getRightAngle() - motors.getLeftAngle();
			new_steering = (motors.getRightAngle() - motors.getLeftAngle() - steering_sync) * 0.05f;
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
			
			int count_scheduler = 0;	// Variable de planificación
			long stabilizerTime = 0;
			double power_motors = 0;
			double turns_power_motors = 0;
			double power_rightmotor = 0;
			double power_leftmotor = 0;
			long last_time = System.currentTimeMillis();
			
			
			/*
			 * Loop 100 Hz (Tiempo comprobado entre XX - XX ms)
			 */
			while(Button.ESCAPE.isUp()) {
				// Código a ejecutar de forma concurrente
				// Determinar condicion para salir del bucle cuando el robot se cae
			
				stabilizerTime = System.currentTimeMillis();
				//lcd.drawString("Tiempo" + (stabilizerTime - last_time) + "   ", 1, 1);
				last_time = stabilizerTime;
				//ctrl.setUpright(true);
	            // runDriveState();
	            
				// Actualización variables del sistema.
				updateVariableState();
				
				/*
				 * Loop 50 Hz (Tiempo comprobado entre XX - XX ms)
				 */
		//		if ((count_scheduler % LOOP_50Hz) == 0){
					// Ponderación de las variables de estado.
					updateWeighingLQR();
					
					// Controlador
					// refspeed += getSpeed() * (dt/1000) * 0.002
					// motorcontroller.setPIDParam(PIDController.PID_SETPOINT, refspeed);
	
	
					power_motors = updateController(error);
					//turns_power_motors = getSteering();
					power_rightmotor = (power_motors + turns_power_motors) * (0.021f / EV3Motor.RADIO_WHEEL);
					power_leftmotor = (power_motors - turns_power_motors) * (0.021f / EV3Motor.RADIO_WHEEL);
					
					if (Segway.STABILIZERLOG) 
						stabilizerlog.println(","+power_rightmotor+","+power_leftmotor);
					            
					if(Math.abs(Psi) < FALLING_DOWN && !getStateStabilizer()){
			            //motors.setPower(pw + ctrl.leftMotorOffset(), pw + ctrl.rightMotorOffset());
					//	motors.setPower(power_leftmotor,power_rightmotor);
					//	System.out.println(power_motors);
			            //motors.setPower(power_motors, power_motors);
										}
					else {
						motors.stop();
						setStateStabilizer(true);
						if  (Math.abs(Psi) > FALLING_DOWN && getStateStabilizer()){
							// Se enfada al caerse
							Button.LEDPattern(8);
						}
						if (Button.UP.isDown()){
							setStateStabilizer(false);
							gyro.reset();
							motors.resetMotors();
							resetController();
						}
					}
		//		}
				//lcd.drawInt((int)power_leftmotor, 2, 7);
				//lcd.drawInt((int)power_rightmotor, 7, 7);
				
				if (count_scheduler++ > SLOWEST_LOOP)
					count_scheduler = 0;
				
				// Se añade 2 ms más para garantizar el tiempo del bucle.
				delay = 2 + (int) ( System.currentTimeMillis()-stabilizerTime ) ;
				
				// Revisar delay2 eliminar
				int delay2 =0;
				if (delay >= dt)
					delay2 = (int) dt;
				else
					delay2 = (int) (dt - delay);
					            
	            // Delay used to stop Gyro being read to quickly. May need to be increase or
	            // decreased depending on leJOS version.
				try {Thread.sleep(delay2);} catch (Exception e) {}
									
			}
			
			if (Segway.GYROLOG) gyro.logClose();
			if (Segway.MOTORLOG) motors.logClose();
			if (Segway.STABILIZERLOG) logClose();
			
			// Se cierra la comunicación con los sensores y actuadores.
			gyro.close();
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
