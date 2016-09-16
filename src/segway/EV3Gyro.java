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
import lejos.hardware.sensor.UARTSensor;
import lejos.utility.Delay;
import lejos.robotics.*;
import segway.FourthOrderFilter;

/**
 * La siguiente clase esta diseñada para trabajar con el giroscopio proporcionado por Lego. Los métodos
 * desarrollados en ella permite calibrar el sensor, y calcula la velocidad de giro y ángulo. 
 * 
 * @author Erevart -- José Emilio Traver
 * @version Agosto 209016
 */
public class EV3Gyro /* extends UARTSensor */ {
	
	private final int sample_calibration = 500;
	private final float max_diff_calibration = 0.5f;
	private final float adaptative_filter_offset = 0.98f;// Weight of older offset values .
	
	// Variables de los sensores
	private float angle = 0f;
	private float angle_rate = 0f;
	private float angle_rate_offset = 0f;
	
	
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
	
	
	/*
	 * Metodos
	 */
 	
	
	 /**
	 * Constructor
	 */
	public EV3Gyro(Port PortGyro)  {
		
//		super(PortGyro,2);
		
		/* Indicar que se activa la inicialización del giroscopio */
		
		// Play tone: frequency 440Hz, volume 10
		// duration 0.1sec, play type 0
		if(Segway.SOUND)
			Sound.playTone(440, 100, 10);
		
		/* Inicialización de atributos */
		Button.LEDPattern(6);
		gyro = new EV3GyroSensor(PortGyro);
		
		filtergyro = new FourthOrderFilter(FourthOrderFilter.CUTOFF_12);
		
		/* Método de inicialización */
		// Se calcula el valor de offset para la calibración del giroscopio.
		calibrateGyro();
		Button.LEDPattern(0);
		
		
		/**
		 * Datalogg
		 */
		if (Segway.GYROLOG) 
			try {gyrolog = new PrintWriter("dataGyro.txt", "UTF-8");
			gyrolog.println("offset,gyro_raw,gyro,angle");}
			catch (FileNotFoundException e1) {e1.printStackTrace();}
			catch (UnsupportedEncodingException e1) {e1.printStackTrace();}
	
	}
	
	/**
	 * Calcula el valor de offset del giroscopio.
	 * @param  	none
	 * @return  none
	 */
	public void calibrateGyro(){
		
		
		// Método de calibración descrito por Lego®.
		// ver http://www.us.lego.com/en-us/mindstorms/community/robot?projectid=96894a3a-45db-48f9-9544-abf66f481b32
		gyro.setCurrentMode("Rate");
		gyro.setCurrentMode("Angle");
		try { Thread.sleep(200);} catch (InterruptedException e) {e.printStackTrace();}
		gyro.setCurrentMode("Rate");
		try { Thread.sleep(3300);} catch (InterruptedException e) {e.printStackTrace();}
		while(!(getRawGyro() >= 0 || getRawGyro() < 0))
			try { Thread.sleep(200);} catch (InterruptedException e) {e.printStackTrace();}
		
					
		// Se inicializa el giroscopio. Se calcula su valor de offset.
		do{
		
		angle_rate_offset = 0f;	
		
		for (int n = 0; n < sample_calibration; n++ ){
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
		if (Segway.GYRODB)
			System.out.println("Gyro offset: "+ angle_rate_offset);
		
		
		} while(Math.abs( (float) ( getRawGyro() ) ) >= max_diff_calibration);	
		
		// Indica que se ha conseguido la calibración
		// Play tone: frequency 440Hz, volume 10
		// duration 0.1sec, play type 0
		//if(Segway.SOUND)
		//	Sound.playTone(440, 200, 10);
		
	}

	public float getRateAngle(){
		
		//angle_rate = filtergyro.filtrate(getRawGyro()-angle_rate_offset);
		angle_rate = getRawGyro()-angle_rate_offset;
		
		// EMA
		//mean = mean * (1f - 0.2f * Stabilizer.dt/1000f) + ((getRawGyro()-angle_rate_offset) * 0.2f * Stabilizer.dt/1000f);
		//angle_rate  = getRawGyro()-angle_rate_offset - mean;
		
		
		/**
		 * Datalog
		 */
		if (Segway.GYROLOG) 
			gyrolog.print(angle_rate_offset+","+angle_rate+","+ (getRawGyro()-angle_rate_offset) );
		
		return angle_rate;
	}
	
	/*
	 * Realiza la integración numérica de velocidad de giro medida con el giroscopio.
	 */

	public float getAngle(){
		

		angle +=  angle_rate * (float) (Stabilizer.dt/1000f);  // (Stabilizer.dt/1000);
	
		/**
		 * Datalogg
		 */
		if (Segway.GYROLOG) 
			gyrolog.println(","+angle);
		
		
		return angle;
	}
	
	public float getRawGyro(){
		
		float[] raw_gyro = new float[1];
		

	//	gyro.getRateMode().fetchSample(raw_gyro, 0);
	//	gyro.fetchSample(raw_gyro, 0);
	//	for (int i = 0; i < 5; i++){
		//raw_gyro[0] = - port.getShort();
			gyro.getRateMode().fetchSample(raw_gyro, 0);
	//		_raw_gyro += raw_gyro[0];
	//	}
		
	//	_raw_gyro /=5;
		
	//	if (raw_gyro[0] > -0.8 && raw_gyro[0] < 0.8)
	//		return 0;
		
				
		/**
		 *  DEBUG
		 */
		if (Segway.GYRODB){
			System.out.println("GYRO: "+ raw_gyro[0] );
		}
		
		//return (float) Math.toRadians(raw_gyro[0]);
		//return (float) _raw_gyro;
		return (float) -raw_gyro[0];
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
   
//   public void close(){
//	   gyro.close();
//   }
	

}
