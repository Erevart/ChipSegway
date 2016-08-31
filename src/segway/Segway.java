package segway;

import java.io.*;

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
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/* Clases propias */
import segway.Stabilizer;

public class Segway {
	
	
	public static boolean SOUND = true;	// Indica si se activan los pitidos.
	
	public static Stabilizer gyroboy;
	
	public static void main(String[] args) {		
		EV3 chip = (EV3) BrickFinder.getDefault();
		TextLCD lcd = chip.getTextLCD();     
		Keys keys = chip.getKeys();
		
		gyroboy = new Stabilizer(chip,SensorPort.S3,chip.getPort("D"),chip.getPort("A"));
		gyroboy.start();
		
		/*
		PrintWriter writer = null;
		try {
			writer = new PrintWriter("the-file-name.txt", "UTF-8");
		} catch (FileNotFoundException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (UnsupportedEncodingException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		writer.println("The first line");
		writer.println("The second line");
		writer.close();	
		*/
		
		int i = 0;
		while (true){
			if (i++ > 8)
				i = 0;
			
	//		Button.LEDPattern(i);
			
			/* Para debug */
			lcd.drawString("Tiempo "+ gyroboy.delay+ "   ", 1, 4);
			lcd.drawString("Angulo: "+ 57*gyroboy.getStabilizerAngle()+ "     ", 1, 3);
			
			lcd.drawString("Var i "+ i+ "   ", 1, 6);
			
			
			if (Button.ESCAPE.isDown()){
				System.out.println("Fuera");
				gyroboy.setStateStabilizer(true);
				return;
			}

			try {Thread.sleep(200);} catch (InterruptedException e) { e.printStackTrace();}
		}
			
	}

}
