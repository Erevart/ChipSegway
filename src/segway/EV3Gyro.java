package segway;

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
	
	private final int sample_calibration = 2000;
	private final float max_diff_calibration = 5.0f;
	
	// Variables del sistema
	private float angle = 0f;
	private float angle_rate = 0f;
	private float angle_rate_offset = 0f;

	
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
		chip = device;
		gyro = new EV3GyroSensor(PortGyro);
		lcd = device.getTextLCD();
		
		filtergyro = new FourthOrderFilter();
		filterangle = new FourthOrderFilter();
		
		/* Método de inicialización */
		// Se calcula el valor de offset para la calibración del giroscopio.
		calibrateGyro();
	
	}
	
	/**
	 * Calcula el valor de offset del giroscopio.
	 * @param  	none
	 * @return  none
	 */
	public void calibrateGyro(){
		
		int n;
		float raw_gyro[] = new float[1];
	
		// Se inicializa el giroscopio. Se calcula su valor de offset.
		do{
		
		n = sample_calibration;
		angle_rate_offset = 0;
		
		while (n-- != 0){
			gyro.getRateMode().fetchSample(raw_gyro, 0);
			angle_rate_offset += raw_gyro[0];
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		angle_rate_offset /= n;
		
		if (Math.abs(angle_rate_offset) >= max_diff_calibration){
			// Indicar que hay que mantener quieto al robot
			// Poner cara de gruñon.
			System.out.println("No te muevas");
		}

		// Para debug.
		System.out.println("Gyro offset: "+angle_rate_offset);
		
		} while(Math.abs(angle_rate_offset) >= max_diff_calibration);	
		
		// Indica que se ha conseguido la calibración

	}
	
	public float getRateAngle(){
		
		float raw_gyro[] = new float[1];
		
		gyro.getRateMode().fetchSample(raw_gyro, 0);
		angle_rate = (float) filtergyro.filtrate(raw_gyro[0]-angle_rate_offset);
				
		return angle_rate;
	}
	
	public float getAngle(float time){
		
		float raw_angle;
		
		raw_angle = angle + angle_rate * time;
		angle = (float) filtergyro.filtrate(raw_angle);
				
		return angle;
	}
	
   /**
    * Reset the gyro angle
    */
   public void resetGyro()
   {
	  
	  // Se resetean los parámetros del giroscopio
	  angle_rate = 0.0f;
	  filtergyro.reset();
	  
      angle = 0.0f;
      filterangle.reset();
      
   }
	

}
