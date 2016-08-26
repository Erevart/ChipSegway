package segway;

import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;

public class Segway {
	
	
	public static void main(String[] args) {
		int numero = 0;
		EV3 ev3 = (EV3) BrickFinder.getDefault();
		TextLCD lcd = ev3.getTextLCD();
		Keys keys = ev3.getKeys();

		lcd.drawString("Segway", 8, 4);
		lcd.drawInt(numero, 4, 5);
		
		keys.waitForAnyPress();
	
		for (int i = 0; i<10; i++)
			numero++;
		
		lcd.drawString("Segway", 8, 4);
		lcd.drawInt(numero, 4, 5);
		keys.waitForAnyPress();
		
		return;
	}

}
