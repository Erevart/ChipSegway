package segway;

import java.beans.Encoder;
import java.io.*;
import java.net.*;
import java.rmi.*;
import java.util.Observable;

import lejos.hardware.Bluetooth;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Key;
import lejos.hardware.Keys;
import lejos.hardware.LocalBTDevice;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.Image;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.port.TachoMotorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.internal.ev3.EV3GraphicsLCD;
import lejos.remote.ev3.RemoteEV3;
import lejos.remote.nxt.BTConnector;
import lejos.remote.nxt.NXTConnection;
import lejos.robotics.EncoderMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/* Clases propias */
import segway.Stabilizer;

public class Segway {
	
	/**
	 * Dataloggers
	 */
	public static boolean GYROLOG = false;
	public static boolean MOTORLOG = false;
	public static boolean STABILIZERLOG = false;
	public static boolean WIFILOG = true;
	
	/**
	 * Debug
	 */
	public static boolean GYRODB = false;
	public static boolean MOTORDB = false;
	public static boolean STABILIZERDB = false;
	public static boolean WIFILOGDB = false;
	
	/**
	 * 
	 * @true desactiva los motores.
	 * @false activa los motores.
	 */
	public static boolean MOTORONDB = false;
	
	/**
	 * Constantes
	 */
	private static final int LOOP_12Hz = 2;
	private static final int LOOP_5Hz = 5;
	private static final int LOOP_2Hz = 10;
	private static final int SLOWEST_LOOP = 2*LOOP_2Hz;
	
	/**
	 * DataLogWifi
	 */
	private static DataLoggerWifi wifilog = null;
	
	
	/**
	 * Complementos de funcionamiento
	 */
	public static boolean SOUND = true;	// Indica si se activan los pitidos.
	public static boolean LED = true;	// Indica si se activan los leds.
	public static boolean LCD = true; 	// Indica si se utiliza la pantalla para mostrar imagenes.
	public static boolean BTH = true;
	public static boolean USC = true;
	
	/**
	 * Clases 
	 */
	public static BluetoothController btcontroller = null;
	public static Stabilizer gyroboy = null;
	public static EV3UltrasonicSensor ultrasonic = null;
	public static EV3ColorSensor colorsensor = null;
	
	private static final int NO_OBSTACLE = 1;
	private static final int TURN_LEFT_OBSTACLE = 2;
	private static final int TURN_RIGHT_OBSTACLE = 3;
	private static final int GOBACK_OBSTACLE = 4;
	
	private static int state_obstacle = NO_OBSTACLE;
		
	
	public static void main(String[] args){
		
		// Se establece la prioridad del hilo principal
		Thread.currentThread().setPriority(8);
		
		// Declaración de variables
		float lcdsteering = 0f;
		float[] distance = new float[1];
		int count_scheduler = 0;
		
		// Declaración de llamadas a objetos.
		EV3 chip = (EV3) BrickFinder.getDefault();	
		
		// Bluetooth
		if (BTH)
			btcontroller = new BluetoothController();
		
		// DataLoggerWifi
		if (WIFILOG)
			wifilog = new DataLoggerWifi();
		  
		// Se muestra una imagen por la LCD.
		if (LCD)
			LegoImage.displayLegoImage("Crazy 1.rgf",EV3GraphicsLCD.TRANS_NONE);
		
		if (USC){
			ultrasonic = new EV3UltrasonicSensor(SensorPort.S4);
			colorsensor = new EV3ColorSensor(SensorPort.S1);
		}
		
		// 	Control de estabilidad
		gyroboy = new Stabilizer(SensorPort.S2,chip.getPort("D"),chip.getPort("A"));
		
		
		// Inicio de procesos.
		gyroboy.start();
		try {Thread.sleep(250);} catch (InterruptedException e) { e.printStackTrace();}
		
		if (BTH)
			btcontroller.start();
		
		// DataLoggerWifi
		if (WIFILOG)
			wifilog.start();
		
		if (USC)
			ultrasonic.enable();
		
		if (LCD){
			LegoImage.displayLegoImage("Winking.rgf",EV3GraphicsLCD.TRANS_NONE);
			try {Thread.sleep(250);} catch (InterruptedException e) { e.printStackTrace();}	
			LegoImage.displayLegoImage("Neutral.rgf",EV3GraphicsLCD.TRANS_NONE);
		}
			

		
		/*
		 * Loop 25 Hz (Tiempo comprobado entre 42 - 52 ms)
		 */
		while (true){
					
			
			/*
			 * Loop 12.5 Hz (Tiempo comprobado entre XX - XX ms)
			 */
			if ((count_scheduler % LOOP_12Hz) == 0){
				
				
				if (WIFILOG){
					wifilog.setDataLog('A', gyroboy.getStabilizerAngle());
					wifilog.setDataLog('G', gyroboy.getStabilizerRateAngle());
					wifilog.setDataLog('P', gyroboy.getStabilizerPosition());
					wifilog.setDataLog('V', gyroboy.getStabilizerSpeed());
					}
					
				if (Segway.LCD)
		        	if (Math.abs(gyroboy.getStabilizerRateAngle()) > 20 && gyroboy.getStateStabilizer())
		        		LegoImage.displayLegoImage("Evil.rgf",EV3GraphicsLCD.TRANS_NONE);	
			}
			
			/*
			 * Loop 5 Hz (Tiempo comprobado entre XX - XX ms)
			 */
			if ((count_scheduler % LOOP_5Hz) == 0){
				
				if (USC){
					ultrasonic.getDistanceMode().fetchSample(distance, 0);
					avoid_obstacle((int)(distance[0]*100));
				}
			}

			/*
			 * Loop 2.5 Hz (Tiempo comprobado entre XX - XX ms)
			 */
			if ((count_scheduler % LOOP_2Hz) == 0){
				if (Segway.LCD){
					lcdsteering = Stabilizer.getSteeringController();
					if (lcdsteering < 0)
						LegoImage.displayLegoImage("Pinch right.rgf",EV3GraphicsLCD.TRANS_NONE);
					else if (lcdsteering > 0)
						LegoImage.displayLegoImage("Pinch left.rgf",EV3GraphicsLCD.TRANS_NONE);
					else if (gyroboy.getStateStabilizer())
						LegoImage.displayLegoImage("Neutral.rgf",EV3GraphicsLCD.TRANS_NONE);
				}
			}
			
			if (Button.ESCAPE.isDown()){
				break;
			}
			
			if (count_scheduler++ > SLOWEST_LOOP)
				count_scheduler = 0;

			try {Thread.sleep(40);} catch (InterruptedException e) { e.printStackTrace();}
		}
		
		// Fin de procesos
		gyroboy.setStateStabilizer(false);
		
		if (LCD){
			try {Thread.sleep(500);} catch (InterruptedException e) { e.printStackTrace();}
			LegoImage.displayLegoImage("Winking.rgf",EV3GraphicsLCD.TRANS_NONE);
			try {Thread.sleep(500);} catch (InterruptedException e) { e.printStackTrace();}
		}
		
		// Cierre de todas las comunicaciones
		if (WIFILOG){
			wifilog.close();
		}
		
		// Fin de procesos
		if (BTH)
			btcontroller.close();
		
		return;
			
	}
	
	
	/**
	 * Evita los obtaculos cercanos
	 */
	public static void avoid_obstacle(int distance_measurament){
		
		if (Float.isNaN(distance_measurament) || distance_measurament == 0){
			if (state_obstacle != NO_OBSTACLE){
				if(BTH)
					btcontroller.setobstacleidentified(false);
				state_obstacle = NO_OBSTACLE;
			}
			return;
		}
		
		if (distance_measurament < 15 && state_obstacle == NO_OBSTACLE){
			state_obstacle = GOBACK_OBSTACLE;
			if(BTH)
				btcontroller.setobstacleidentified(true);
			Button.LEDPattern(4);
		}
		
		switch(state_obstacle){
		
			case GOBACK_OBSTACLE:
				Stabilizer.setSteering(0f);
				Stabilizer.setSpeed(-6f);
				if (distance_measurament > 20)
					state_obstacle = TURN_LEFT_OBSTACLE;
			break;
			
			case TURN_LEFT_OBSTACLE:
				Stabilizer.setSteering(20f);
				Stabilizer.setSpeed(0f);
				Button.LEDPattern(7);
				if (distance_measurament > 35){
					Button.LEDPattern(1);
					state_obstacle = NO_OBSTACLE;
					if(BTH)
						btcontroller.setobstacleidentified(false);
				}
				else 
					state_obstacle = TURN_RIGHT_OBSTACLE;
				break;
				
			case TURN_RIGHT_OBSTACLE:
				Stabilizer.setSteering(-20f);
				if (distance_measurament > 35 ){
					Button.LEDPattern(1);
					state_obstacle = NO_OBSTACLE;
					if(BTH)
						btcontroller.setobstacleidentified(false);
				}
				break;
			case NO_OBSTACLE:
				break;
			default:
				Stabilizer.setSteering(0f);
				Stabilizer.setSpeed(0f);
				state_obstacle = NO_OBSTACLE;
				if(BTH)
					btcontroller.setobstacleidentified(false);
				break;
		
		}
		return;
	}
	

}
