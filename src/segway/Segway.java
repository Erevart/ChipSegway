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
	public static boolean GYROLOG = false;
	public static boolean MOTORLOG = false;
	public static boolean STABILIZERLOG = true;
	public static boolean WIFILOG = true;
	
	/**
	 * Debug
	 */
	public static boolean GYRODB = false;
	public static boolean MOTORDB = false;
	public static boolean STABILIZERDB = false;
	public static boolean WIFILOGDB = true;
		
	
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
		
		// 
		EV3 chip = (EV3) BrickFinder.getDefault();	
		
		// LCD
		TextLCD lcd = chip.getTextLCD();    
		
		// 	Control de estabilidad
		gyroboy = new Stabilizer(chip,SensorPort.S2,chip.getPort("D"),chip.getPort("A"));
    	gyroboy.start();
		
		// Servidor TCP
		if (WIFILOG){
			wifilog = new DataLoggerWifi();
			wifilog.start();
		} 

		
       
		double i = 0;
		while (true){
			if (i++ > 8)
				i = 0;
			
	//		Button.LEDPattern(i);
			
			/* Para debug */
			/*
			lcd.drawString("Tiempo "+ gyroboy.delay+ "   ", 1, 4);
			lcd.drawString("Angulo: "+ gyroboy.getStabilizerAngle()+ "     ", 1, 3);
			lcd.drawString("Gyro: "+ gyroboy.getStabilizerRateAngle()+ "     ", 1, 5);
			
			lcd.drawString("Var i "+ i+ "   ", 1, 6);
			*/
			
			Button.LEDPattern((int) i);
			if (WIFILOG){
				wifilog.setDataLog('A', gyroboy.getStabilizerAngle());
			}
			
			if (Button.ESCAPE.isDown()){
				break;
			}

			try {Thread.sleep(40);} catch (InterruptedException e) { e.printStackTrace();}
		}
		
		
		
		// Cierre de todas las comunicaciones
		if (WIFILOG)
			wifilog.close();
		
		System.out.println("Fuera");
		gyroboy.setStateStabilizer(true);
		
		return;
			
	}


}
