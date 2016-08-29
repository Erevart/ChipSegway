package segway;

import lejos.hardware.Audio;
import lejos.hardware.Button;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.utility.Delay;
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
	private final float max_diff_calibration = 0.2f;
	
	// Variables de los sensores
	private double angle = 0;
	private double angle_rate = 0;
	private double angle_rate_offset = 0;
	private long lastAngleTime = 0;
	

	
	// Definición de sensores y hardware
	private EV3GyroSensor gyro;
	private TextLCD lcd;
	private EV3 chip;
	
	// Filtro
	FourthOrderFilter filtergyro;
	FourthOrderFilter filterangle;
 	
	
	
	 /**
	 * Constructor
	 */
	public EV3Gyro(EV3 device, Port PortGyro) {
		
		/* Inicialización de atributos */
		Button.LEDPattern(4);
		chip = device;
		gyro = new EV3GyroSensor(PortGyro);
		lcd = device.getTextLCD();
		
		filtergyro = new FourthOrderFilter();
		filterangle = new FourthOrderFilter();
		
		/* Método de inicialización */
		// Se calcula el valor de offset para la calibración del giroscopio.
		calibrateGyro();
		Button.LEDPattern(0);
	
	}
	
	/**
	 * Calcula el valor de offset del giroscopio.
	 * @param  	none
	 * @return  none
	 */
	public void calibrateGyro(){
		
		int n;
		double new_offset = 0;
		float raw_gyro[] = new float[1];
	
		// Se inicializa el giroscopio. Se calcula su valor de offset.
		do{
		
		n = sample_calibration;
		//new_offset = angle_rate_offset;
		angle_rate_offset = 0;
		
		
		while (n-- != 0){
			gyro.getRateMode().fetchSample(raw_gyro, 0);
			angle_rate_offset += Math.toRadians(raw_gyro[0]);
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		angle_rate_offset /= sample_calibration;
		
		if (Math.abs(angle_rate_offset-new_offset) >= max_diff_calibration){
			// Indicar que hay que mantener quieto al robot
			// Poner cara de gruñon.
			
			System.out.println("No te muevas");
			Button.LEDPattern(2);
			//gyro.reset();
			
		}

		// Para debug.
		System.out.println("Gyro offset: "+angle_rate_offset);
		
		} while(Math.abs(angle_rate_offset - new_offset) >= max_diff_calibration);	
		
		// Indica que se ha conseguido la calibración
		lcd.clear();

	}
	
	public double getRateAngle(){
		
		float[] raw_gyro = new float[1];
		
		gyro.getRateMode().fetchSample(raw_gyro, 0);
		angle_rate = (double) filtergyro.filtrate(Math.toRadians(raw_gyro[0])-angle_rate_offset);
		
		return angle_rate;
	}
	
	/*
	 * Reliaza la integración numérica de velocidad de giro medida con el giroscopio.
	 */
	
	public double getAngle(){
		
		double raw_angle;
		long currentTime = System.currentTimeMillis();
		
		if (currentTime != lastAngleTime){
			raw_angle = angle +  angle_rate * ( (double) (currentTime - lastAngleTime) )/1000;
			//angle = filterangle.filtrate(raw_angle);
			angle = raw_angle;
		}
		
		//float[] raw_gyro = new float[1];
		
		//gyro.getAngleMode().fetchSample(raw_gyro, 0);
		
		lastAngleTime = currentTime;
				
		//return (double) Math.toRadians(raw_gyro[0]);
		return angle;
	}
	
   /**
    * Reset the gyro angle
    */
   public void reset()
   {
	  
	  // Se resetean los parámetros del giroscopio
	  angle_rate = 0.0f;
	  lastAngleTime = 0;
	  filtergyro.reset();
	  
      angle = 0.0f;
      filterangle.reset();
      
   }
	

}
