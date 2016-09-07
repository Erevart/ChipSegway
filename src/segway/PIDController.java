package segway;

import lejos.utility.Delay;

public class PIDController {
	
    /**
     * Proportional term ID
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_KP = 0;
    /**
     * Integral term ID. The <code>I</code> accumulator is an <tt>int</tt> so any decimal places are rounded in the calc: 
     * <code>I += Ki * error * dt;</code>
     * 
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_KI = 1;
    /**
     * Derivitive term ID
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_KD = 2;
    /**
     * The Ramping Exponential value ID. Used for output (MV) ramping/attenuation which determines ramp shape. 1.0=linear, Set to 0.0 to disable 
     * output ramping. Larger values &gt;= 1 create steeper ramping curves. 0&lt;PID_RAMP_POWER&lt;1 will invert the curve
     * which will cause exponential MV amplification the closer we get to the SP (would this ever be useful?).
     * 
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_RAMP_POWER = 3;
    /**
     * The Ramping Threshold value ID. Used for output ramping. When the PID Manipulated Variable (MV) is within this range (-+), output ramping is
     * applied to MV before it is returned from <code>{@link #doPID}</code>. The value passed to <tt>setPIDParam()</tt>
     *     is cast to an <tt>int</tt>.
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_RAMP_THRESHOLD = 4;
    /**
     * The deadband value ID. Used for output clipping. If MV within +- this range relative to zero, MV of zero is returned.
     * Set to zero to effectively disable. This is useful to avoid hunting around the SP when there is a lot
     * of slop in whatever the controller is controlling i.e. gear & link lash. The value passed to <tt>setPIDParam()</tt>
     *     is cast to an <tt>int</tt>. Using deadband is process actuator/control-specific and by definition, decreases accuracy 
     *     of reaching the <code>SP</code>.
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_DEADBAND = 5;
    
    /**
     *     The MV high limit cutoff value ID. Use for high limit cutoff for Manipulated Variable (<code>MV</code>). Set to a large value to 
     *     effectively disable. This is applied to <code>MV</code> before any ramping. Default is 900 at instantiation. The value 
     *     passed to <tt>setPIDParam()</tt>
     *     is cast to an <tt>int</tt>.
     *  
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_LIMITHIGH = 6;
    
    /**
     * The MV low limit cutoff value ID. Use for low limit cutoff for Manipulated Variable (<code>MV</code>). Set to a large negative value to 
     *     effectively disable. This is applied to <code>MV</code> before any ramping. Default is -900. The value passed to <tt>setPIDParam()</tt>
     *     is cast to an <tt>int</tt>.
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_LIMITLOW = 7;
    
    /**
     * The Setpoint value ID. This is the value the PID controller works toward by changing MV is response to the error 
     * when <tt>doPID()</tt> is called. The value passed to <tt>setPIDParam()</tt>
     *     is cast to an <tt>int</tt>.
     * @see #setPIDParam
     * @see #getPIDParam
     * @see #doPID
     */
    public static final int PID_SETPOINT = 8;
    
    /**
     * The Integral low limit cutoff value ID. Use for limiting accumulation of <code>I (Ki * error * dt)</code> less than a defined value. Set to zero
     *     to disable. Default is 0. Setting this clears the <code>I</code> term accumulator.
     * <P>    This is one methodology to manage integral windup.
     * @see #setPIDParam
     * @see #getPIDParam
     * @see #freezeIntegral
     * @see #PID_I_LIMITHIGH
     */
    public static final int PID_I_LIMITLOW = 9;
    
    /**
     * The Integral high limit cutoff value ID. Use for limiting accumulation of <code>I (Ki * error * dt)</code> greater than a defined value. Set to zero
     *     to disable. Default is 0. Setting this clears the <code>I</code> term accumulator. The value passed to <tt>setPIDParam()</tt>
     *     is cast to an <tt>int</tt>.
     * <P>    This is one methodology to manage integral windup.
     * @see #setPIDParam
     * @see #getPIDParam
     * @see #freezeIntegral
     * @see #PID_I_LIMITLOW
     */
    public static final int PID_I_LIMITHIGH = 10;

    /** The integral accumulator <code>I</code> value. Read-only.Calling <tt>setPIDParam()</tt> with this is ignored. The <tt>I</tt> value is the
     * accumulator for <code>Ki * error * dt</code>. 
     */
    public static final int PID_I = 11;
    
    /** The process variable (<code>PV</code>) value. Read-only.Calling <tt>setPIDParam()</tt> with this is ignored. The <tt>PV</tt> value is the
     * last value passed to <code>doPID()</code>.
     * 
     * @see #doPID
     */
    public static final int PID_PV = 12;
    
    /**
     * Intregration sample term ID
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_DT = 13;

    // Our default Constants for the PID controller 
    private float Kp=1.0f;          		// proportional value determines the reaction to the current error
    private float Ki=0.0f;     				// integral value determines the reaction based on the sum of recent errors
    private float Kd=0.0f;       			// derivative value determines the reaction based on the rate at which the error has been changing
    private int highLimit = 100;            // assuming control of motor speed and thereby max would be 900 deg/sec
    private int lowLimit = -highLimit;
    private double previous_error = 0;
    private int deadband = 0;
    private double dt = 0;                     // cycle time, ms
    private long cycleTime=0;               // used to calc the time between each call (dt) to doPID()
    private double setpoint;                   // The setpoint to strive for
    private double error;                      // proportional term
    private double integral = 0;               // integral term
    private double derivative;               // derivitive term
    private int integralHighLimit = 0;
    private int integralLowLimit = 0;
    private boolean integralLimited = false;
    private boolean disableIntegral = false;
    private float power = 0;
    private int rampThresold = 0;
    private double rampExtent = 1;
    private int msdelay;
//    private Logger dataLogger=null;
    private int cycleCount=0;
    private double PV;
	
	 /**
	 * Constructor por defecto. 
	 */
	public PIDController(){}
	
     /**
     * Construct a PID controller instance using passed setpoint (SP) and millisecond delay (used before returning from a call to
     * <code>doPID()</code>).
     * @param setpoint The goal of the MV 
     * @param msdelay The delay in milliseconds. Set to 0 to disable any delay.
     * @see #doPID
     * @see #setDelay
     */
    public PIDController(int setpoint, int msdelay) {
        this.setpoint = setpoint;
        this.msdelay = msdelay;
    }

    /**
     * Set PID controller parameters.
     * @param paramID What parameter to set. See the constant definitions for this class.
     * @param value The value to set it to. Note that some values are cast to <tt>int</tt> depending on the particular <tt>paramID</tt> value used.
     * @see #getPIDParam
     */
    public void setPIDParam(int paramID, float value) {
        switch (paramID) {
            case PIDController.PID_KP:
                this.Kp = value;
                break;
            case PIDController.PID_KI:
                this.Ki = value;
                break;
            case PIDController.PID_KD:
                this.Kd = value;
                break;
            case PIDController.PID_RAMP_POWER:
                this.power = value;
                rampExtent = Math.pow(this.rampThresold, this.power);
                break;
            case PIDController.PID_RAMP_THRESHOLD:
                this.rampThresold = (int)value;
                if (this.rampThresold==0) break;
                rampExtent = Math.pow(this.rampThresold, this.power);
                break;
            case PIDController.PID_DEADBAND:
                this.deadband = (int)value;
                break;
            case PIDController.PID_LIMITHIGH:
                this.highLimit = (int)value;
                break;
            case PIDController.PID_LIMITLOW:
                this.lowLimit = (int)value;
                break;
            case PIDController.PID_SETPOINT:
                this.setpoint = value;
                this.cycleTime = 0;
                break;
            case PIDController.PID_I_LIMITLOW:
                this.integralLowLimit = (int)value;
                this.integralLimited = (this.integralLowLimit!=0);    
                break; 
            case PIDController.PID_I_LIMITHIGH:
                this.integralHighLimit = (int)value;
                this.integralLimited = (this.integralHighLimit!=0);
                break; 
            case PIDController.PID_DT:
                this.dt = (int)value;
                break;     
            default:
                return;
        }
        // zero the Ki accumulator
        integral = 0;
    }

    /** Get PID controller parameters.
     * @param paramID What parameter to get. See the constant definitions for this class.
     * @return The requested parameter value
     *  @see #setPIDParam
     */
    public double getPIDParam(int paramID) {
        double retval =0;
        switch (paramID) {
            case PIDController.PID_KP:
                retval=this.Kp;
                break;
            case PIDController.PID_KI:
                retval=this.Ki;
                break;
            case PIDController.PID_KD:
                retval=this.Kd;
                break;
            case PIDController.PID_RAMP_POWER:
                retval=this.power;
                break;
            case PIDController.PID_RAMP_THRESHOLD:
                retval=this.rampThresold;
                break;
            case PIDController.PID_DEADBAND:
                retval = this.deadband;
                break;
            case PIDController.PID_LIMITHIGH:
                retval = this.highLimit;
                break;
            case PIDController.PID_LIMITLOW:
                retval = this.lowLimit;
                break;
            case PIDController.PID_SETPOINT:
                retval = this.setpoint;
                break; 
            case PIDController.PID_I_LIMITLOW:
                retval = this.integralLowLimit ;
                break; 
            case PIDController.PID_I_LIMITHIGH:
                retval = this.integralHighLimit;
                break; 
            case PID_I:
                retval = this.integral;
                break;
            case PID_PV:
                retval = this.PV;
                break;
            default:
        }
        return retval;
    }

    /** Freeze or resume integral accumulation. If frozen, any pre-existing integral accumulation is still used in the MV calculation. This
     * is useful for disabling the integral function until the PV has entered the controllable region [as defined by your process
     * requirements].
     * <P>This is one methodology to manage integral windup. This is <tt>false</tt> by default at instantiation.
     * 
     * @param status <tt>true</tt> to freeze, <tt>false</tt> to thaw
     * @see #isIntegralFrozen
     */
    public void freezeIntegral(boolean status){
        this.disableIntegral = status;
    }

    /**
     * 
     * @return <code>true</code> if the integral accumulation is frozen
     * @see #freezeIntegral
     */
    public boolean isIntegralFrozen() {
        return this.disableIntegral;
    }
    
    /**
     * Do the PID calc for a single iteration. Your implementation must provide the delay between calls to this method if you have
     * not set one with <code>setDelay()</code> or in the constructor.
     * @param processVariable The PV value from the process (sensor reading, etc.). 
     * @see #setDelay
     * @return The Manipulated Variable <code>MV</code> to input into the process (motor speed, etc.)
     */
    public int doPID(double processVariable){
        int outputMV;
        int delay=0;
        this.PV = processVariable;
        
        /*
        if (this.cycleTime==0) {
            this.cycleTime = System.currentTimeMillis();
            return 0;
        }
        */
        error = setpoint - processVariable;
        error = Math.abs(error)<=deadband?0:error;
        if (!disableIntegral) integral += Ki * error * dt; 
        if (integralLimited){
            if (integral>integralHighLimit) integral = integralHighLimit;
            if (integral<integralLowLimit) integral = integralLowLimit;
        }
        if (dt != 0)
        	derivative = ((float)(error - previous_error))/dt;
        outputMV = (int)(Kp*error + integral + Kd*derivative);
        
        if (outputMV>highLimit) outputMV=highLimit;
        if (outputMV<lowLimit) outputMV=lowLimit;
        previous_error = error;
        outputMV=rampOut(outputMV);

       /*
        // delay the difference of desired cycle time and actual cycle time
        if (this.msdelay>0) {
            delay = this.msdelay-((int)(System.currentTimeMillis() - this.cycleTime)); // desired cycle time minus actual time
            if (delay>0) {
                Delay.msDelay(delay);
            }
        }
        */
        cycleCount++;
        // global time it took to get back to this statement
        //dt = (double)(System.currentTimeMillis() - this.cycleTime)/1000;
        //this.cycleTime = System.currentTimeMillis();
        return outputMV;
    }

    /** Register a <code>NXTDataLogger</code> instance to log the PID variables. If the logger instance is in
     * cached mode, the headers must not have been set before calling this method or <code>false</code> is returned. 
     * <code>PIDController</code> will
     * set the column headers and log values on every call to <code>doPID()</code>.
     * <p>
     * This is useful when using the NXJChartingLogger tool to visualize the PID response by monitoring the internal
     * variables.
     * @param dataLogger A <code>NXTDataLogger</code> instance in realtime or cached logging mode.
     * @return <code>true</code> if successful, <code>false</code> otherwise.
     * @see NXTDataLogger
     * @see #deregisterDataLogger
     */
    /*
    public boolean registerDataLogger(Logger dataLogger){
        LogColumn[] logColumns = {
            new LogColumn("SP",LogColumn.DT_INTEGER),
            new LogColumn("MV",LogColumn.DT_INTEGER),
            new LogColumn("PV",LogColumn.DT_INTEGER),
            new LogColumn("Integral",LogColumn.DT_INTEGER),
            new LogColumn("Kp*error",LogColumn.DT_FLOAT),
            new LogColumn("Kd*derivative",LogColumn.DT_FLOAT,2),
            new LogColumn("error",LogColumn.DT_INTEGER),
            new LogColumn("dt",LogColumn.DT_INTEGER),
        };
        try {
            dataLogger.setColumns(logColumns);
        } catch (UnsupportedOperationException e){
            return false;
        }
        this.dataLogger=dataLogger;
        return true;
    }
    */
    /** De-register the registered <code>NXTDataLogger</code>. 
     * @return The <code>NXTDataLogger</code> that was registered, <code>null</code> if no logger has been registered.
     * @see #registerDataLogger
     */
  /*
    public Logger deregisterDataLogger(){
        Logger tempDL=this.dataLogger;
        this.dataLogger=null;
        return tempDL;
    }
   */ 
    private int rampOut(int ov){
        if (power==0 || rampThresold==0) return ov;
        if (Math.abs(ov)>rampThresold) return ov;
        int workingOV;
        workingOV=(int)(Math.pow(Math.abs(ov), power) / rampExtent * rampThresold);
        return (ov<0)?-1*workingOV:workingOV;
    }

    /** 
     * Set the desired delay before <code>doPID()</code> returns. Set to zero to effectively disable.
     * 
     * @param msdelay Delay in milliseconds
     * @see #getDelay
     */
    public void setDelay(int msdelay) {
        this.msdelay = msdelay;
    }
    
    /**
     * Returns the <code>doPID()</code> timing delay. 
     * 
     * @return The delay set by <code>setDelay()</code>
     * @see #setDelay
     */
    public int getDelay() {
        return this.msdelay;
    }
}
