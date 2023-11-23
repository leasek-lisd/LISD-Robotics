package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;


public class Claw {
  private Servo clawServo;
  private double clawServoPos;
  private double clawServoRatio = .005;
  private static double clawServoOpen=.4;
  private static double clawServoClosed=.75;
    // todo: write your code here
    public Claw(Servo inClawServo){
      clawServo = inClawServo;
    }
    
    
    public void closeClawSlow(double y){
    double adj = y * clawServoRatio;
    clawServoPos = Math.max(clawServoPos - adj, clawServoClosed); 
    setClawPosition(clawServoPos);
  }  
  public void closeClaw(){
    clawServoPos = clawServoClosed;
    setClawPosition(clawServoPos);
  }
  
  public void openClawSlow(double y){
    double adj = y * clawServoRatio;
    clawServoPos = Math.min(clawServoPos + adj, clawServoOpen); 
    setClawPosition(clawServoPos);
    
  }  
  public void openClaw(){
    setClawPosition(clawServoOpen);
  }
  
  public void setClawPosition(double newPos){
    clawServoPos = newPos;
    clawServo.setPosition(clawServoPos);
  }
}