package segway;

import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.utility.Delay;

import segway.FourthOrderFilter;

public class EV3Gyro {
	
	private final int sample_calibration = 2000;
	private final float max_diff_calibration = (float) Math.toRadians(5);
	
	// Variables del sistema
	private float angle = 0f;
	private float angle_offset = 0f;
	private float angle_rate = 0f;
	private float angle_rate_offset = 0f;
	private float speed = 0f;
	private float steering = 0f;
	private float max_acc = 0f;
	
	// Definición de sensores
	EV3GyroSensor gyro;
	
	// Filtro
	FourthOrderFilter filtergyro;
 	
	/* Revisar todo de aqui en adelante */
	private boolean isrun = true;
	private long previous_time = 0;
	private TextLCD lcd;
	private EV3 chip;
	
	
	 /**
	 * Constructor
	 */
	public EV3Gyro(EV3 device, Port PortGyro) {
		
		this.chip = device;
		
		this.gyro = new EV3GyroSensor(PortGyro);
		filtergyro = new FourthOrderFilter();
		
		lcd = device.getTextLCD();
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
			angle_rate_offset += Math.toRadians(raw_gyro[0]);
			Delay.msDelay(5);
		}
		
		angle_rate_offset /= n;
		
		if (angle_rate_offset >= max_diff_calibration){
			// Indicar que hay que mantener quieto al robot
			// Poner cara de gruñon.
		}

		// Para debug.
		System.out.println("Gyro offset: "+angle_rate_offset);
		
		} while(angle_rate_offset >= max_diff_calibration);	

	}
	
	public float getRateAngle(){
		
		float raw_gyro[] = new float[1];
		
		gyro.getRateMode().fetchSample(raw_gyro, 0);
		angle_rate = (float) filtergyro.filtrate(Math.toRadians(raw_gyro[0])-angle_rate_offset);
				
		
		return angle_rate;
	}
	

}
