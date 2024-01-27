package org.firstinspires.ftc.teamcode;


public class PidController {
  private double Kp;
  private double Ki;
  private double Kd;
  private double setPoint;
  private double error;
  private double lastError;
  private double reset;
  private double maxOut;
  private double minOut;
  private double maxError;
    
  public PidController(double kP, double kI, double kD){
      Kp=kP;
      Ki=kI;
      Kd=kD;
      error = 0;
      lastError=0;
      reset = 0;
      
      maxOut = .25;
      minOut = -.25;
  }
  
  public PidController(double kP, double kI, double kD, double min, double max){
      Kp=kP;
      Ki=kI;
      Kd=kD;
      error = 0;
      lastError=0;
      reset = 0;
      maxOut = max;
      minOut = min;
  }

  
  public double getP(){
      return Kp*error;
  }
  
  public double getI(){
      reset += error;
      
      return Ki*reset;
      
  }    
  
  public double getD(){
    double returnValue = Kd* (error - lastError);
    lastError = returnValue;
    return returnValue;
  }
  
  
  public double getPidOutput(double processVariable){
      error = setPoint-processVariable;
      if(Math.abs(error) > maxError){
        return Math.min(maxOut,Math.max(minOut,getP()+getI()+getD()));  
      }
      return 0;
  }
  
  
  
  public void setSetPoint(double newSet){
    setPoint=newSet;
    
  }
  
}