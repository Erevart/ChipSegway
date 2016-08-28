/**
 * 
 */
package segway;

import lejos.ev3.tools.EV3Console;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Key;
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
	
	// Definición de parámetros del robot
	private float diameter_wheel = 58; 					// Diametro de las ruedas (mm). 
	private float radio_wheel = diameter_wheel/2000;	// Radio de las ruedas (m)
	
	
	// Sintonización variables del controlador 
	private final float dt = 22f; 	// Tiempo de muestreo (ms) // En lego dt = ( 22 - 2) / 1000
	private float kp = 0.5f;		// Ganancia proporcional
	private float ki = 11f;			// Ganancia integral
	private float kd = 0.005f;		// Ganancia derivativa
	/* Consignas */
	private float refbalance = 0f;
	private float refturn = 0f;
	private float refspeed = 0f;
	
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
	private double Psi = 0;
	private double PsiDot = 0;
	private double Phi = 0;
	private double PhiDot = 0;
	private double steering = 0;
	private double max_acc = 0;
	
	// Definición de controladores
	PIDController motorcontroller;
	PIDController pendulumcontroller;
	
	// Definición de sensores y actuadores
	EV3Gyro gyro;
	EV3Motor motors;
	
	/* Revisar todo de aqui en adelante */
	private boolean isrun = true;
	private long previous_time = 0;
	private TextLCD lcd;
	private EV3 chip;
	
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
		motorcontroller = new PIDController();
		pendulumcontroller = new PIDController();
		
		lcd = device.getTextLCD();
		
		/* Inicialización de objetos */
		// Se establece los parametros del controlador del péndulo
		// los cuales influyen a su vez en la velocidad de los motores
		pendulumcontroller.setPIDParam(PIDController.PID_SETPOINT, 0);
		pendulumcontroller.setPIDParam(PIDController.PID_KP, kp);
		pendulumcontroller.setPIDParam(PIDController.PID_KI, ki);
		pendulumcontroller.setPIDParam(PIDController.PID_KD, kd);
		
		// Inicializar demás parámetros.
		
		
		// Se establece los parametros del controlador de los motores
		motorcontroller.setPIDParam(PIDController.PID_KP, kp);
		motorcontroller.setPIDParam(PIDController.PID_KI, ki);
		motorcontroller.setPIDParam(PIDController.PID_KD, kd);
		
	}	
	
	/*
	 * Actualiza el valor de las variables de estado del sistema.
	 */
	private void updateVariableState(){
		
        // Actualización variables del sistema
		PsiDot = gyro.getRateAngle();
        Psi = gyro.getAngle(dt);
        
        // ctrl.tiltAngle() is used to drive the robot forwards and backwards
        Phi = motors.getAngle();// - ctrl.tiltAngle();
        PhiDot = motors.getAngularVelocity();
        
        return;
	}
	
	/*
	 * Se pondera el valor de las variables de estado del sistema a partir 
	 * de los pesos proprocionado por el algoritmo de optimización LQR.
	 */
	private void updateWeighingLQR(){
		
        Psi *= Kpsi;
        PsiDot *= Kpsidot;
        
        Phi *= Kphi;
        PhiDot *= Kphidot;
		
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
			
			long stabilizerTime;
			double power_motors;
			
			// Código a ejecutar de forma concurrente
			// Determinar condicion para salir del bucle cuando el robot se cae
			while(gyro.getRateAngle() < 60){
				
				stabilizerTime = System.currentTimeMillis();
				//ctrl.setUpright(true);
	            // runDriveState();
	            
				// Actualización variables del sistema.
				updateVariableState();
				
				// Ponderación de las variables de estado.
				updateWeighingLQR();
				
				// Controlador
				
				motorcontroller.setPIDParam(PIDController.PID_SETPOINT, 0);
				
				power_motors = motorcontroller.doPID(Phi+PhiDot);
				
				power_motors += pendulumcontroller.doPID(Psi+PsiDot);
				            
	            //motors.setPower(pw + ctrl.leftMotorOffset(), pw + ctrl.rightMotorOffset());
	            motors.setPower(power_motors, power_motors);
	            
	            // Delay used to stop Gyro being read to quickly. May need to be increase or
	            // decreased depending on leJOS version.
	            try {
	            	Thread.sleep((long) (dt - (stabilizerTime-System.currentTimeMillis())) );
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}
	
	/**
	 * 
	 * @function startStabilizer
	 * @brief 	Invoca e inicia el Thread Stabilizer para estabilizar el robot o
	 * 			o lograr su desplazamiento sin perder el equilibrio.
	 * @param - none
	 * @return 	none
	 */
	public void start() {
		new Thread(new StabilizerThread()).start() ;
	}

}
