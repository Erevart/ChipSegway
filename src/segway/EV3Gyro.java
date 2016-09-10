package segway;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

import lejos.hardware.Audio;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
import lejos.robotics.*;
import segway.FourthOrderFilter;

/**
 * La siguiente esta diseñada para trabajar con el giroscopio proporcionado por Lego. Los métodos
 * desarrollados en ella permite calibrar el sensor, y calcula la velocidad de giro y ángulo. 
 * 
 * @author Erevart -- José Emilio Traver
 * @version Agosto 209016
 */
public class EV3Gyro {
	
	private final int sample_calibration = 500;
	private final float max_diff_calibration = 1f;
	private final float adaptative_filter_offset = 0.98f;// Weight of older offset values .
	
	// Variables de los sensores
	private double angle = 0;
	private double angle_rate = 0;
	private double angle_rate_offset = 0;
	
	
	// Definición de sensores y hardware
	private EV3GyroSensor gyro;
	private TextLCD lcd;
	private EV3 chip;
	
	// Filtro
	FourthOrderFilter filtergyro;
	
	/**
	 * Datalloger
	 * Indicar que dato se guardarán
	 */
	PrintWriter gyrolog = null;
 		
	 /**
	 * Constructor
	 */
	public EV3Gyro(EV3 device, Port PortGyro) {
		
		/* Indicador que se inicio la inicialización del giroscopio */
		
		// Play tone: frequency 440Hz, volume 10
		// duration 0.1sec, play type 0
		if(Segway.SOUND)
			Sound.playTone(440, 100, 10);
		
		/* Inicialización de atributos */
		Button.LEDPattern(6);
		chip = device;
		gyro = new EV3GyroSensor(PortGyro);
		lcd = device.getTextLCD();
		
		filtergyro = new FourthOrderFilter(FourthOrderFilter.CUTOFF_12);
		
		/* Método de inicialización */
		// Se calcula el valor de offset para la calibración del giroscopio.
		calibrateGyro();
		Button.LEDPattern(0);
		
		
		/**
		 * Datalogg
		 */
		if (Segway.GYROLOG) 
			try {gyrolog = new PrintWriter("dataGyro.txt", "UTF-8");}
			catch (FileNotFoundException e1) {e1.printStackTrace();}
			catch (UnsupportedEncodingException e1) {e1.printStackTrace();}
	
	}
	
	/**
	 * Calcula el valor de offset del giroscopio.
	 * @param  	none
	 * @return  none
	 */
	public void calibrateGyro(){
		
		int n;
		
		// Método de calibración descrito por Lego®.
		// ver http://www.us.lego.com/en-us/mindstorms/community/robot?projectid=96894a3a-45db-48f9-9544-abf66f481b32
		gyro.setCurrentMode("Rate");
/*		gyro.setCurrentMode("Angle");
		try { Thread.sleep(200);} catch (InterruptedException e) {e.printStackTrace();}
		gyro.setCurrentMode("Rate");
*/		try { Thread.sleep(3300);} catch (InterruptedException e) {e.printStackTrace();}
		while(!(getRawGyro() >= 0 || getRawGyro() < 0))
			try { Thread.sleep(200);} catch (InterruptedException e) {e.printStackTrace();}
		
					
		// Se inicializa el giroscopio. Se calcula su valor de offset.
		do{
		
		n = sample_calibration;
		angle_rate_offset = 0;	
		
		while (n-- != 0){
			angle_rate_offset +=getRawGyro();
			try { Thread.sleep(5);} catch (InterruptedException e) {e.printStackTrace();}
			
		}
		
		angle_rate_offset /= sample_calibration;
		
		if (Math.abs( (float) ( getRawGyro() - angle_rate_offset ) ) >= max_diff_calibration){
			// Indicar que hay que mantener quieto al robot
			// Poner cara de gruñon.
	
			System.out.println("No te muevas");
			Button.LEDPattern(5);
		}

		/**
		 *  DEBUG
		 */
		if (Segway.GYRODB){
			System.out.println("Gyro offset: "+ angle_rate_offset*57);
		}
		
		} while(Math.abs( (float) ( getRawGyro() - angle_rate_offset) ) >= max_diff_calibration);	
		
		// Indica que se ha conseguido la calibración
		// Play tone: frequency 440Hz, volume 10
		// duration 0.1sec, play type 0
		//if(Segway.SOUND)
		//	Sound.playTone(440, 200, 10);
		
	}
	
	public double getRateAngle(){
		
	//	angle_rate = filtergyro.filtrate(getRawGyro()-angle_rate_offset);	
		
		/**
		 * Datalogg
		 */
		if (Segway.GYROLOG) 
			gyrolog.print(angle_rate_offset+","+angle_rate+","+ (getRawGyro()-angle_rate_offset) );
		
		//return angle_rate;
		return getRawGyro();
	}
	
	/*
	 * Realiza la integración numérica de velocidad de giro medida con el giroscopio.
	 */

	public double getAngle(){
		
		angle = angle +  angle_rate * (double) (Stabilizer.dt/1000);
		
       // angle = angle + (double) (Stabilizer.dt/1000) 
       // 		* (angle_rate[0] + 2*angle_rate[1] + 2*angle_rate[2] + angle_rate[3])/6;
		
		// 0.017 = 1 deg
		//if ((angle > -0.09f) && (angle < 0.09f) )
		//	angle_rate_offset = angle_rate_offset * adaptative_filter_offset + (1.0f - adaptative_filter_offset)*angle_rate;
				
		/**
		 * Datalogg
		 */
		if (Segway.GYROLOG) 
			gyrolog.println(","+angle);
		
		return angle;
	}
	
	public double getRawGyro(){
		
		float[] raw_gyro = new float[1];

		gyro.getRateMode().fetchSample(raw_gyro, 0);
		
	//	if (raw_gyro[0] > -0.2 && raw_gyro[0] < 0.2)
	//		return 0;
		
				
		/**
		 *  DEBUG
		 */
		if (Segway.GYRODB){
			System.out.println("GYRO: "+ raw_gyro[0] );
		}
		
		//return (double) -Math.toRadians(raw_gyro[0]);
		return (double) -raw_gyro[0];
	}
	
   /**
    * Reset the gyro angle
    */
   public void reset()
   {
	  // Se resetean los parámetros del giroscopio
	  angle_rate = 0.0f;
	  filtergyro.reset();
	  
      angle = 0.0f;
   }
   
   public void logClose(){
	   gyrolog.flush();
	   gyrolog.close();
   }
   
   public void close(){
	   gyro.close();
   }
	

}
