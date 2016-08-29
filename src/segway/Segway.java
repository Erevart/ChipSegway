package segway;

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
	
	public static Stabilizer threadStabilizer;
	
	public static void main(String[] args) {		
		EV3 chip = (EV3) BrickFinder.getDefault();
		TextLCD lcd = chip.getTextLCD();     
		Keys keys = chip.getKeys();
		threadStabilizer = new Stabilizer(chip,SensorPort.S2,chip.getPort("D"),chip.getPort("A"));
		
		threadStabilizer.start();
		
		
		int i = 0;
		while (true){
			if (i++ > 9)
				i = 0;
			Button.LEDPattern(i);
			
			/* Para debug */
			/*
			lcd.drawString("Gyro       ", 2, 2);
			lcd.drawInt((int)Math.toDegrees(threadStabilizer.PsiDot), 7, 2);
			lcd.drawString("Angulo      ", 1, 3);
			lcd.drawInt((int)Math.toDegrees(threadStabilizer.Psi), 7, 3);
			*/

			
			if (Button.ESCAPE.isDown()){
				threadStabilizer.setStateStabilizer(true);
				return;
			}
			
			try {Thread.sleep(100);} catch (InterruptedException e) { e.printStackTrace();}
		}
			
	}

}
