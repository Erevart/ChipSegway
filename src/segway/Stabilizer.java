/**
 * 
 */
package segway;

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
	private final float dt = 10f; 	// Tiempo de muestreo (ms) // En lego dt = ( 22 - 2) / 1000
	private float kp = 0.5f;		// Ganancia proporcional
	private float ki = 11f;			// Ganancia integral
	private float kd = 0.005f;		// Ganancia derivativa
	/* Consignas */
	private double error = 0;
	private float refspeed = 0f;
	public boolean stateStabilizer = false;
	
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
	public double Psi = 0;
	public double PsiDot = 0;
	private double Phi = 0;
	private double PhiDot = 0;
	private double steering = 0;
	private double max_acc = 0;
	
	// Definición de controladores
	PIDController PID;
	
	// Definición de sensores y actuadores
	EV3Gyro gyro;
	EV3Motor motors;
	
	/* Revisar todo de aqui en adelante */
	private boolean isrun = true;
	private long previous_time = 0;
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
		setStateStabilizer(true);
		
	}	
	
	/*
	 * Actualiza el valor de las variables de estado del sistema.
	 */
	private void updateVariableState(){
		
        // Actualización variables del sistema
		PsiDot = gyro.getRateAngle();
        Psi = gyro.getAngle();
        
        // ctrl.tiltAngle() is used to drive the robot forwards and backwards
        Phi = motors.getRobotPosition();// - ctrl.tiltAngle();
        PhiDot = motors.getRobotSpeed();
        
        return;
	}
	
	/*
	 * Se pondera el valor de las variables de estado del sistema a partir 
	 * de los pesos proprocionado por el algoritmo de optimización LQR.
	 */
	private void updateWeighingLQR(){
		
		// refspeed += getSpeed() * (dt/1000) * 0.002
		
		error = Psi * Kpsi +
				PsiDot * Kpsidot +
				( Phi - refspeed) * Kphi +
				PhiDot * Kphidot;
		
	}
	
	public boolean getStateStabilizer(){
		
		return stateStabilizer;
	}
	
	public void setStateStabilizer(boolean state){
		
		stateStabilizer = state;
	}
	
	public double getSteering(){
		// Posible lock
		double new_steering = 0;
	
		
		if (new_steering == 0){
		
			double steering_sync = (steering != 0)?(motors.getRightAngle() - motors.getLeftAngle()):0;
			new_steering = (motors.getRightAngle() - motors.getLeftAngle() - steering_sync) * 0.05;
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
			double power_motors = 0;
			double turns_power_motors = 0;;
	
			do {
				// Código a ejecutar de forma concurrente
				// Determinar condicion para salir del bucle cuando el robot se cae
				while(Math.abs(Psi) < falling_down && getStateStabilizer()){
					
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
					
					power_motors = PID.doPID(error);
					// turns_power_motor = getSteering();
					// power_rightmotor = (power_motors - turns_power_motor) * (0.0021 / EV3Motor.radio_wheel)
					// power_leftmotor = (power_motors + turns_power_motor) * (0.0021 / EV3Motor.radio_wheel)
					            
		            //motors.setPower(pw + ctrl.leftMotorOffset(), pw + ctrl.rightMotorOffset());
		   //         motors.setPower(power_rightmotor, power_leftmotors);
										

					if (System.currentTimeMillis()-stabilizerTime < dt)
						delay = (int) (dt - (System.currentTimeMillis()-stabilizerTime) );
					else
						delay = (int) dt;	
						            
		            // Delay used to stop Gyro being read to quickly. May need to be increase or
		            // decreased depending on leJOS version.
					try {Thread.sleep(delay);} catch (Exception e) {}
					
				}
				motors.stop();
				setStateStabilizer(false);
				
				while (Math.abs(Psi) > falling_down && !getStateStabilizer()){
					updateVariableState();
				//	Sound.beep();
					Button.LEDPattern(7);
					if (Button.ESCAPE.isDown())
						return;
					try { Thread.sleep((long) dt);} catch (InterruptedException e) {}
				}
			
				if (Button.UP.isDown()){
					setStateStabilizer(true);
					gyro.reset();
					motors.resetMotors();
					updateVariableState();
				} else if (Button.ESCAPE.isDown())
					break;
				
				try { Thread.sleep((long) dt);} catch (InterruptedException e) {}
				
			}while(true);
			
			return;
		}
	}
	
	/**
     * Invoca e inicia el Thread Stabilizer para estabilizar el robot o
	 * 			o lograr su desplazamiento sin perder el equilibrio.
	 * @param - none
	 * @return 	none
	 */
	public void start() {
		new Thread(new StabilizerThread()).start() ;
	}

}
