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
	private float diameter_wheel = 55; 					// Diametro de las ruedas (mm). 
	private float radio_wheel = diameter_wheel/2000;	// Radio de las ruedas (m)
	
	
	// Sintonización variables del controlador 
	private final float dt = 22f; 	// Tiempo de muestreo (ms) // En lego dt = ( 22 - 2) / 1000
	private float kp = 0.5f;		// Ganancia proporcional
	private float ki = 11f;			// Ganancia integral
	private float kd = 0.005f;		// Ganancia derivatica
	private float refpos = 0f;
	
	// Parametros de movimiento
	private float gain_angular_velocity = 1.3f; // Ganancia de velocidad angular. 
	private float gain_angle = 25f;
	private float gain_motor_speed = 75f;
	private float gain_motor_position = 350f; 
	
	// Variables del sistema
	private float angle = 0f;
	private float angle_rate = 0f;
	private float speed = 0f;
	private float steering = 0f;
	private float max_acc = 0f;
	
	// Definición de sensores
	EV3Gyro gyro;
 	
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
	public Stabilizer(EV3 device, Port PortGyro) {
		
		this.chip = device;
		
		this.gyro = new EV3Gyro(device,PortGyro);
		
		lcd = device.getTextLCD();
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
			// Código a ejecutar de forma concurrente
			int i = 0;
			while(isrun){
				if (i++ > 9)
					i = 0;
				lcd.drawString("Thread 1", 6, 4);
				lcd.drawInt(i, 6, 5);
				try {
					Thread.sleep(5);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				if (Button.ESCAPE.isDown())
					isrun = false;
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
