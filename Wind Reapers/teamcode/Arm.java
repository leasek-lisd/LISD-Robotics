package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
public class Arm {
  
  private DcMotor armMotorL;
  private DcMotor armMotorR;
  private Servo elbowServo;
  private double elbowServoPos;
  private double elbowServoRatio;
  private double armPowerReduction = .45;
 // private TouchSensor limitSensor;
  // public Arm(DcMotor inArmMotorL, DcMotor inArmMotorR, Servo inElbowServo, TouchSensor inSensor){
  public Arm(Servo inElbowServo,HardwareMap hardwareMap){
   // armMotorL=inArmMotorL;
   // armMotorR=inArmMotorR;
  //  limitSensor = inSensor;
    elbowServo = inElbowServo;
    elbowServoPos = 0;
    elbowServoRatio = .003;
    armMotorL= hardwareMap.get(DcMotor.class, "armMotorLeftC0");
    armMotorR = hardwareMap.get(DcMotor.class, "armMotorRightC1");
    
    //armMotorL.setDirection(DcMotor.Direction.REVERSE);
    armMotorR.setDirection(DcMotor.Direction.REVERSE);
    
    armMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }
    
  
    // todo: write your code here
  
  public void move(double y){
    double power = y;
    
      armMotorL.setPower(y*armPowerReduction);  
      armMotorR.setPower(y*armPowerReduction);
      
   
  
  }
  
  public void moveSteps(int steps, double speed){
    // 38 steps per rev
    // 80:1 gear ratio

    armMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotorL.setTargetPosition(steps);
    armMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armMotorL.setPower(speed);
  }
  
  public void moveElbow(double y){
    double adj = y * elbowServoRatio;
    double newPos = Math.min(Math.max(elbowServoPos + adj, .15), .75); 
    setElbowPosition(newPos);
    elbowServoPos=newPos;
  }
  
  public void setElbowPosition(double newPos){
    elbowServoPos = newPos;
    elbowServo.setPosition(elbowServoPos);
  }
  
  public void setElbowServoRatio(double newRatio){
    elbowServoRatio = newRatio;
    
  }
  
}
