/**
 * 
 */
package segway;

import java.io.File;
import java.io.FileNotFoundException;
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
public final class Stabilizer {
		
	// Sintonización variables del controlador 
	private final int falling_down = (int) (Math.PI/3); // Umbral de inclinación a partir de cual no se ejecuta el controlador. 
	private final int dt = 20; 	// Tiempo de muestreo (ms) // En lego dt = ( 22 - 2) / 1000
	private float kp = 0.5f;		// Ganancia proporcional
	private float ki = 11f;			// Ganancia integral
	private float kd = 0.005f;		// Ganancia derivativa
	/* Consignas */
	private double error = 0;
	private float refpos = 0f;
	private boolean stateStabilizer = false;
	
	// Ponderación de las variables del sistema (Variables Lego)
	private float Kpsidot = 1.3f; // Ganancia de velocidad angular. 
	private float Kpsi = 25f;
	private float Kphidot = 75f;
	private float Kphi  = 350f; 
	
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
	private float Psi = 0;
	private float PsiDot = 0;
	private float Phi = 0;
	private float PhiDot = 0;
	private float steering = 0;
	private float new_steering = 0;
	private float steering_sync = 0;
	
	// Definición de controladores
	private PIDController PID;
	
	// Definición de sensores y actuadores
	public EV3Gyro gyro;
	private EV3Motor motors;
	
	// Mutex - Lock
	private Lock lock_drivecontrol;
	private Lock lock_stabilizer;
	
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
		motors = new EV3Motor(portleftMotor,portrightMotor);
		PID = new PIDController();
		
		lcd = device.getTextLCD();
		
		/* Inicialización de objetos */
		// Se establece los parametros del controlador 
		PID.setPIDParam(PIDController.PID_SETPOINT,0);
		PID.setPIDParam(PIDController.PID_KP, kp);
		PID.setPIDParam(PIDController.PID_KI, ki);
		PID.setPIDParam(PIDController.PID_KD, kd);
	
		
		/* Atributos de permiso y acceso */
		lock_drivecontrol = new ReentrantLock();
		lock_stabilizer = new ReentrantLock();
		
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
        Phi = motors.getRobotPosition();// - ctrl.tiltAngle();
        PhiDot = motors.getRobotSpeed();
        
        lock_stabilizer.unlock();
        
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
	
	/*
	 * Se pondera el valor de las variables de estado del sistema a partir 
	 * de los pesos proprocionado por el algoritmo de optimización LQR.
	 */
	private void updateWeighingLQR(){
		
		// refpos += getSpeed() * (dt/1000) * 0.002
		error = Kpsi * Psi +
				Kpsidot * PsiDot +
				Kphi * ( Phi - refpos) +
				Kphidot * PhiDot;
		
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
	
	private float getSteering(){
		// Posible lock
		
		float new_steering = 0;
		
		lock_drivecontrol.lock();
		// Se actualiza el valor de la variable global de la clase a la local del método.
		new_steering = this.new_steering;
		lock_drivecontrol.unlock();
		
		if (new_steering == 0){
		
			steering_sync = (steering != 0)?(motors.getRightAngle() - motors.getLeftAngle()):0;
			new_steering = (motors.getRightAngle() - motors.getLeftAngle() - steering_sync) * 0.05f;
			return new_steering;
			
		}
		else {
			steering = (new_steering > 50)?50:new_steering;
			steering = (new_steering < -50)?-50:new_steering;
			return steering / 2;
		}
 

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
	
		public void run () {
			
			long stabilizerTime = 0;
			float power_motors = 0;
			float turns_power_motors = 0;
			float power_rightmotor = 0;
			float power_leftmotor = 0;
			long last_time = System.currentTimeMillis();
			

			while(true) {
				// Código a ejecutar de forma concurrente
				// Determinar condicion para salir del bucle cuando el robot se cae
				
					
					stabilizerTime = System.currentTimeMillis();
					lcd.drawString("Tiempo" + (stabilizerTime - last_time) + "   ", 1, 1);
					last_time = stabilizerTime;
					//ctrl.setUpright(true);
		            // runDriveState();
		            
					// Actualización variables del sistema.
					updateVariableState();
					
					// Ponderación de las variables de estado.
					updateWeighingLQR();
					
					// Controlador
					// refspeed += getSpeed() * (dt/1000) * 0.002
					// motorcontroller.setPIDParam(PIDController.PID_SETPOINT, refspeed);
					
					power_motors = PID.doPID(error);
			//		turns_power_motors = getSteering();
					power_rightmotor = (power_motors - turns_power_motors) * (0.021f / EV3Motor.radio_wheel);
					power_leftmotor = (power_motors + turns_power_motors) * (0.021f / EV3Motor.radio_wheel);
					            
					if(Math.abs(Psi) < falling_down && !getStateStabilizer()){
			            //motors.setPower(pw + ctrl.leftMotorOffset(), pw + ctrl.rightMotorOffset());
						//motors.setPower(power_rightmotor, power_leftmotor);
			            //motors.setPower(power_motors, power_motors);
						Button.LEDPattern(1);
					}
					else {
						motors.stop();
						setStateStabilizer(true);
						if  (Math.abs(Psi) > falling_down && getStateStabilizer()){
							// Se enfada al caerse
							Button.LEDPattern(8);
						}
					
						if (Button.ESCAPE.isDown()){
							gyro.logClose();
							break;
						}
						else if (Button.UP.isDown()){
							setStateStabilizer(true);
							gyro.reset();
							motors.resetMotors();
						}
					}
					
					//lcd.drawInt((int)power_motors, 2, 7);
					//lcd.drawInt((int)power_rightmotor, 7, 7);
					
					// Se añade 2 ms más para garantizar el tiempo del bucle.
					delay = 2 + (int) ( System.currentTimeMillis()-stabilizerTime ) ;
					
					int delay2 =0;
					if (delay >= dt)
						delay2 = (int) dt;
					else
						delay2 = (int) (dt - delay);
						            
		            // Delay used to stop Gyro being read to quickly. May need to be increase or
		            // decreased depending on leJOS version.
					try {Thread.sleep(delay2);} catch (Exception e) {}
									
			}
			//return;
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
