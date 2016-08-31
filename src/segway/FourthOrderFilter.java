package segway;

/**
 * Filtro IIR para el filtrado de señales digitales y analgícos.
 * Parámetros obtenidos a partir de la función "cheby2(4,60,12.5/50)" de Matlab.
 * @author Erevart -- José Emilio Traver
 *
 */
public class FourthOrderFilter {

	/*
	final private double _b0 = 0.001893594048567;
	final private double _b1 = -0.002220262954039;
	final private double _b2 = 0.003389066536478;
	final private double _b3 = -0.002220262954039;
	final private double _b4 = 0.001893594048567;
	  
	final private double _a1 = -3.362256889209355;
	final private double _a2 = 4.282608240117919;
	final private double _a3 = -2.444765517272841;
	final private double _a4 = 0.527149895089809;
	*/
	// cheby2(4,80,7.5/50)
	
	final private double _b0 = 0.000133915564256199;
	final private double _b1 = -0.000350496206949809;
	final private double _b2 =  0.000471439054629189;
	final private double _b3 = -0.000350496206949810;
	final private double _b4 = 0.000133915564256199;
	  
	final private double _a1 = -3.789736516006818 ;
	final private double _a2 = 5.391036665652461;
	final private double _a3 = -3.411557330314591;
	final private double _a4 = 0.810295458438189;
	
	/* Variables de entrada para el diseño del filtro */
	private double inputbuffer[] = new double[4];
	
	/* Variables de salida para el diseño del filtro */
	private double outputbuffer[] = new double[4];
	
	/**
	 * Constructor
	 */	
	public FourthOrderFilter(){
		reset();
	}
	
	public float filtrate(float data){
		
		 float output;
		  
		  output = (float) (_b0 * data   + 
		           _b1 * inputbuffer[0]  + 
		           _b2 * inputbuffer[1]  +
		           _b3 * inputbuffer[2]  +
		           _b4 * inputbuffer[3]  -
		           _a1 * outputbuffer[0] -
		           _a2 * outputbuffer[1] -
		           _a3 * outputbuffer[2] -
		           _a4 * outputbuffer[3]);

           inputbuffer[3] = inputbuffer[2];
           inputbuffer[2] = inputbuffer[1];
           inputbuffer[1] = inputbuffer[0];
           inputbuffer[0] = data;
		  
           outputbuffer[3] = outputbuffer[2];
           outputbuffer[2] = outputbuffer[1];
           outputbuffer[1] = outputbuffer[0];
           outputbuffer[0] = output;
		    
		  return output;
	}
	
	public double filtrate(double data){
		
		  double output;
		  
		  output = _b0 * data   + 
		           _b1 * inputbuffer[0]  + 
		           _b2 * inputbuffer[1]  +
		           _b3 * inputbuffer[2]  +
		           _b4 * inputbuffer[3]  -
		           _a1 * outputbuffer[0] -
		           _a2 * outputbuffer[1] -
		           _a3 * outputbuffer[2] -
		           _a4 * outputbuffer[3];

          inputbuffer[3] = inputbuffer[2];
          inputbuffer[2] = inputbuffer[1];
          inputbuffer[1] = inputbuffer[0];
          inputbuffer[0] = data;
		  
          outputbuffer[3] = outputbuffer[2];
          outputbuffer[2] = outputbuffer[1];
          outputbuffer[1] = outputbuffer[0];
          outputbuffer[0] = output;
		    
		  return output;
	}
	
	public void reset(){
		for (int i = 0; i < 4; i++){
			inputbuffer[i] = 0;
			outputbuffer[i] = 0;
		}
		return;
	}
}
