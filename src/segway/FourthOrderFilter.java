package segway;

/**
 * Filtro IIR para el filtrado de señales digitales y analgícos.
 * Parámetros obtenidos a partir de la función "cheby2" de Matlab.
 * @author Erevart -- José Emilio Traver
 *
 */
public class FourthOrderFilter {
	
	/** 
	 * Selecciona los parámetros del filtro IIR para una frecuencia de corte 
	 * de 12.5 Hz, para una frecuencia de muestreo de 100 Hz. "cheby2(4,60,12.5/50)"
	 **/
	public static final int CUTOFF_12 = 0;
	
	/** 
	 * Selecciona los parámetros del filtro IIR para una frecuencia de corte 
	 * de 22.5 Hz, para una frecuencia de muestreo de 100 Hz. "cheby2(4,60,22.5/50)"
	 **/
	public static final int CUTOFF_22 = 1;
	
	
	// cheby2(4,60,12.5/50)
	final private double CUTOFF_12_b[] = {0.001893594048567f, -0.002220262954039f, 0.003389066536478f, -0.002220262954039f, 0.001893594048567f};
	final private double CUTOFF_12_a[] = {-3.362256889209355f, 4.282608240117919f, -2.444765517272841f, 0.527149895089809f};
	
	// cheby2(4,80,22.5/50)
	final private double CUTOFF_22_b[] = {0.005756353346780f, 0.006760424571151f , 0.010311233686422f,0.006760424571151f,0.005756353346780f};
	final private double CUTOFF_22_a[] = {-2.709225974919292f,2.899883100028628f,-1.424647249159285f,0.269334913572232f};
	
	
	// cheby2(4,80,7.5/50)
	/*
	final private double _b0 = 0.000133915564256199;
	final private double _b1 = -0.000350496206949809;
	final private double _b2 =  0.000471439054629189;
	final private double _b3 = -0.000350496206949810;
	final private double _b4 = 0.000133915564256199;
	  
	final private double _a1 = -3.789736516006818 ;
	final private double _a2 = 5.391036665652461;
	final private double _a3 = -3.411557330314591;
	final private double _a4 = 0.810295458438189;
	*/
	
	/* Variables de entrada para el diseño del filtro */
	private double inputbuffer[] = new double[4];
	private double coeff_b[] = new double[5];
	
	/* Variables de salida para el diseño del filtro */
	private double outputbuffer[] = new double[4];
	private double coeff_a[] = new double[5];
	
	/**
	 * Constructor
	 */	
	public FourthOrderFilter(int type_filter){
		reset();
		setparam(type_filter);
	}
	
	public float filtrate(float data){
		
		 float output;
		  
		  output = (float) (coeff_b[0] * data   + 
				   coeff_b[1] * inputbuffer[0]  + 
		           coeff_b[2] * inputbuffer[1]  +
		           coeff_b[3] * inputbuffer[2]  +
		           coeff_b[4] * inputbuffer[3]  -
		           coeff_a[0] * outputbuffer[0] -
		           coeff_a[1] * outputbuffer[1] -
		           coeff_a[2] * outputbuffer[2] -
		           coeff_a[3] * outputbuffer[3]);

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
		  
		  output = coeff_b[0] * data   + 
				   coeff_b[1] * inputbuffer[0]  + 
		           coeff_b[2] * inputbuffer[1]  +
		           coeff_b[3] * inputbuffer[2]  +
		           coeff_b[4] * inputbuffer[3]  -
		           coeff_a[0] * outputbuffer[0] -
		           coeff_a[1] * outputbuffer[1] -
		           coeff_a[2] * outputbuffer[2] -
		           coeff_a[3] * outputbuffer[3];

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
	
	private void setparam(int type_filter){
		
		switch(type_filter){
		
			case CUTOFF_12:
				coeff_b = CUTOFF_12_b;
				coeff_a = CUTOFF_12_a;						
			break;
			
			case CUTOFF_22:
				coeff_b = CUTOFF_22_b;
				coeff_a = CUTOFF_22_a;
			break;
			
			default:
			break;			
		}
	}
	
	public void reset(){
		for (int i = 0; i < 4; i++){
			inputbuffer[i] = 0f;
			outputbuffer[i] = 0f;
		}
		return;
	}
}
