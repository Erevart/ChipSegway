package segway;

import java.beans.Encoder;
import java.io.*;
import java.net.*;
import java.rmi.*;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Key;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.port.TachoMotorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.remote.ev3.RemoteEV3;
import lejos.robotics.EncoderMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/* Clases propias */
import segway.Stabilizer;

public class Segway {
	
	/**
	 * Dataloggers
	 */
	public static boolean GYROLOG = true;
	public static boolean MOTORLOG = false;
	public static boolean STABILIZERLOG = true;
	public static boolean WIFILOG = false;
	
	/**
	 * Debug
	 */
	public static boolean GYRODB = false;
	public static boolean MOTORDB = false;
	public static boolean STABILIZERDB = false;
	public static boolean WIFILOGDB = false;
	
	/**
	 * Constantes
	 */
	private static final int LOOP_12Hz = 2;
	private static final int SLOWEST_LOOP = 2*LOOP_12Hz;
	/**
	 * DataLogWifi
	 */
	private static DataLoggerWifi wifilog = null;
	
	
	/**
	 * Complementos de funcionamiento
	 */
	public static boolean SOUND = true;	// Indica si se activan los pitidos.
	
	/**
	 * Clases 
	 */
	public static Stabilizer gyroboy;
	
	
	public static void main(String[] args){
		
		// Declaración de variables
		int count_scheduler = 0;
		
		// Declaración de llamadas a objetos.
		EV3 chip = (EV3) BrickFinder.getDefault();	
		
		// LCD
		TextLCD lcd = chip.getTextLCD();    
		
		// 	Control de estabilidad
		gyroboy = new Stabilizer(SensorPort.S2,chip.getPort("D"),chip.getPort("A"));
		gyroboy.setStateStabilizer(true);
    	gyroboy.start();
		
		// DataLoggerWifi
		if (WIFILOG){
			wifilog = new DataLoggerWifi();
			wifilog.start();
		} 
		
		// Espera a la estabilización de variables y sensores
		try {Thread.sleep(800);} catch (InterruptedException e) { e.printStackTrace();}
		
		// Se inicia el controal
//		gyroboy.setStateStabilizer(true);
		
		float i = 0;
		/*
		 * Loop 25 Hz (Tiempo comprobado entre 42 - 52 ms)
		 */
		while (true){
			
			if (i++ > 8)
				i = 0;
	//		lcd.drawString("               " ,1, 5);
	//		lcd.drawInt((int) (t-t2), 8, 5);
	//		lcd.drawString("Tiempo" ,1, 5);
					
			/* Para debug */
			
		//	lcd.drawString("Tiempo "+ gyroboy.delay+ "   ", 1, 4);
		//	lcd.drawString("Angulo: "+ gyroboy.getStabilizerAngle()+ "     ", 1, 3);
		/*	lcd.drawString("Gyro: "+ gyroboy.getStabilizerRateAngle()+ "     ", 1, 5);
			
			lcd.drawString("Var i "+ i+ "   ", 1, 6);
		*/
			
			Button.LEDPattern((int) i);
			
			/*
			 * Loop 12.5 Hz (Tiempo comprobado entre XX - XX ms)
			 */
			/*
			if ((count_scheduler % LOOP_12Hz) == 0){
				if (WIFILOG){
					wifilog.setDataLog('A', gyroboy.getStabilizerAngle());
					wifilog.setDataLog('G', gyroboy.getStabilizerRateAngle());
					wifilog.setDataLog('P', gyroboy.getStabilizerPosition());
					wifilog.setDataLog('V', gyroboy.getStabilizerSpeed());/*
					wifilog.setDataLog('L', gyroboy.getStabilizerAngle());
					wifilog.setDataLog('R', gyroboy.getStabilizerAngle());
					wifilog.setDataLog('C', gyroboy.getStabilizerAngle());
					wifilog.setDataLog('R', gyroboy.getStabilizerAngle());
					wifilog.setDataLog('I', gyroboy.getStabilizerAngle());
					wifilog.setDataLog('D', gyroboy.getStabilizerAngle());*
				}
			}
			*/
			if (Button.ESCAPE.isDown()){
				break;
			}
			
			if (count_scheduler++ > SLOWEST_LOOP)
				count_scheduler = 0;

			try {Thread.sleep(40);} catch (InterruptedException e) { e.printStackTrace();}
		}
		
		
		
		// Cierre de todas las comunicaciones
		if (WIFILOG){
			wifilog.setDataLog('#',0);
			wifilog.close();
			wifilog.stop();
		}
		
		System.out.println("Fuera");
		gyroboy.setStateStabilizer(false);
		
		return;
			
	}


}
