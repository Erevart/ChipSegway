package segway;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.TachoMotorPort;
import lejos.robotics.EncoderMotor;
import lejos.robotics.RegulatedMotor;

/**
 * La clase EV3Motor está diseñada para trabajar con los motores. Los métodos
 * desarrollados en ella permiten establecer la velocidad y ángulo de los motores. En base al
 * codigo realizado por Steven Jan Witzand.
 * 
 * @author Erevart -- José Emilio Traver
 * @version Agosto 2016
 */

class EV3Motor
{

   private EncoderMotor leftMotor;
   private EncoderMotor rightMotor;
	
   // Definición de parámetros del robot
   public static final float diameter_wheel = 58f; 					// Diametro de las ruedas (mm). 
   public static final float radio_wheel = diameter_wheel/2000f;		// Radio de las ruedas (m)
   private float position = 0f;
   private float last_position = 0f;
   
   // Filtro
   private FourthOrderFilter filterwheelspeed;
   
   // Sinusoidal parameters used to smooth motors
   private float sin_x = 0.0f;
   private final float sin_speed = 0.1f;
   private final float sin_amp = 20.0f;
   
	/**
	 * Datalloger
	 * Indicar que dato se guardarán
	 */
	PrintWriter motorlog = null;

   /**
    * MotorController constructor.
    * 
    * @param leftMotor
    *           The GELways left motor.
    * @param rightMotor
    *           The GELways right motor.
    */
   public EV3Motor(Port portleftMotor,Port portrightMotor)
   {
	   Button.LEDPattern(5);
      leftMotor = new UnregulatedMotor(portleftMotor);
      leftMotor.resetTachoCount();

      rightMotor = new UnregulatedMotor(portrightMotor);
      rightMotor.resetTachoCount();
      Button.LEDPattern(0);
      
      filterwheelspeed = new FourthOrderFilter();
      
		// erase array
		for (int i = 0; i < max_index; i++) {
			enc_val [i] = 0;
		}
      
  	/**
  	 * Datalloger
  	 * Indicar que dato se guardarán
  	 */
      if (Segway.MOTORLOG)
	      try {motorlog = new PrintWriter("dataMotor.txt", "UTF-8");}
	      catch (FileNotFoundException e1) {e1.printStackTrace();}
	      catch (UnsupportedEncodingException e1) {e1.printStackTrace();}
   }

   /**
    * Method is used to set the power level to the motors required to keep it upright. A
    * dampened sinusoidal curve is applied to the motors to reduce the rotation of the
    * motors over time from moving forwards and backwards constantly.
    * 
    * @param leftPower
    *           A float used to set the power of the left motor. Maximum value depends on
    *           battery level but is approximately 815. A negative value results in motors
    *           reversing.
    * @param rightPower
    *           A float used to set the power of the right motor. Maximum value depends on
    *           battery level but is approximately 815. A negative value results in motors
    *           reversing.
    */
   public void setPower(float leftPower, float rightPower)
   {
      sin_x += sin_speed;
      int pwl = (int) (leftPower);//+ Math.sin(sin_x) * sin_amp);
      int pwr = (int) (rightPower);// - Math.sin(sin_x) * sin_amp);

      leftMotor.setPower(pwl);
      if (pwl < 0) {
         leftMotor.backward();
      } else if (pwl > 0) {
         leftMotor.forward();
      } else {
         leftMotor.stop();
      }

      rightMotor.setPower(pwr);
      if (pwr < 0) {
         rightMotor.backward();
      } else if (pwr > 0) {
         rightMotor.forward();
      } else {
         rightMotor.stop();
      }
   }

   /**
    * getAngle returns the average motor angle of the left and right motors
    * 
    * @return A float of the average motor angle of the left and right motors in degrees.
    */
   public float getAngle()
   {
      return (float) (leftMotor.getTachoCount() + rightMotor.getTachoCount() ) / 2.0f;
   }

   /**
    * getAngle returns the average motor velocity of the left and right motors
    * 
    * @return a float of the average motor velocity of the left and right motors in
    *         degrees.
    */
   public float getAngularVelocity()
   {
      return 0; //(float) (leftMotor.getSpeed() + rightMotor.getSpeed()) / 2.0f;
   }
   
   
   /**
    * getSpeed devuelve la velocidad media de ambos motores.
    * 
    * @return a float con el valor medio de la velocidad de las ruedas 
    * 
    */
   
	private static final int max_index = 7;
	float[] enc_val = new float[max_index];
	int enc_index = 0;
	
   public float getSpeed(){
	   
	   float speed_raw = 0f; 
	   
	   speed_raw = (position - last_position) / ((float)Stabilizer.dt/1000f);
	   
	   enc_index++;

		if (max_index <= enc_index)
			enc_index = 0;

		int compare_index = enc_index + 1;
		if (max_index <= compare_index)
			compare_index = 0;

		enc_val[enc_index] = (leftMotor.getTachoCount() /*+ rightMotor.getTachoCount() )/2.0f */) ;
					//System.out.println (enc_val [enc_index] + " " + enc_val[compare_index] + " " + max_index + " " + Stabilizer.dt);
					System.out.println(speed_raw);
					
		  if (Segway.MOTORLOG)
		  motorlog.println(","+speed_raw+","+((enc_val [enc_index] - enc_val [compare_index]) / (max_index * Stabilizer.dt/1000)	));

		
		
		return ((enc_val [enc_index] - enc_val [compare_index]) / (max_index * Stabilizer.dt/1000)	);
	   
	   
	   /* REVISAR QUITAR O DEJAR FILTRO */
	   
    //  return 0f;// filterwheelspeed.filtrate(((float)leftMotor.getSpeed() + (float)rightMotor.getSpeed()) * (Math.toRadians(1) * radio_wheel / 2));
    		  
   }
   
   /**
    * getRobotPosition devuele la posición de los motores calculada a partir de los encoders en m. 
    * 
    * @return a float con la posición de las ruedas.
    */
   public float getPosition(){
	   
	   last_position = position;
	   
	   position = (leftMotor.getTachoCount() + rightMotor.getTachoCount()) * ( (float) Math.toRadians(1) * radio_wheel / 2f);
	   
	   if (Segway.MOTORDB)
		   System.out.println ("ML "+leftMotor.getTachoCount()+" MR " + rightMotor.getTachoCount());
	   if (Segway.MOTORLOG)
		   motorlog.print(position);

	   return position;
   }
   
   /**
    * getRightAngle devuelve la posición del motor derecho en grados.
    * 
    * @return a float con la posición del motor derecho.
    */
   public float getRightAngle(){
	   return (float) rightMotor.getTachoCount();
   }
   
   
   /**
    * getLeftAngle devuelve la posición del motor izquierdo en grados.
    * 
    * @return a float con la posición del motor izquierdo.
    */
   public float getLeftAngle()
   {
      return (float) leftMotor.getTachoCount();
   }
   

   /**
    * reset the motors tacho count
    */
   public void resetMotors()
   {
      leftMotor.resetTachoCount();
      rightMotor.resetTachoCount();
   }

   /**
    * stop both motors from rotating
    */
   public void stop()
   {
      leftMotor.stop();
      rightMotor.stop();
   }
   
   /**
    * float stop both motors from rotating
    */
   public void flt()
   {
      leftMotor.flt();
      rightMotor.flt();
   }
   
   public void logClose(){
	   motorlog.close();
   }
	
   
}