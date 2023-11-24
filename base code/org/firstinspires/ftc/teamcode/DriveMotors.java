package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.ExecutionException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class DriveMotors extends Wheels  {
    
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
  
    public DriveMotors(
        DcMotor inFrontLeftDrive,
        DcMotor inFrontRightDrive,
        DcMotor inBackLeftDrive,
        DcMotor inBackRightDrive
    ){
        
      frontLeftDrive = inFrontLeftDrive; 
      frontRightDrive = inFrontRightDrive;
      backLeftDrive = inBackLeftDrive;
      backRightDrive = inBackRightDrive;
    
    }

// The next four methods are where you setup your wheels
// add code that makes your wheel roll forward with a positive value for power
// "Forward" meaning that the top of the wheel rolls toward the front of the bot

  public void turnFrontLeft(double power){
    frontLeftDrive.setPower(power);
  }
  
  public void turnFrontRight(double power){
    frontRightDrive.setPower(power);
  }
  
  public void turnBackLeft(double power){
    backLeftDrive.setPower(power);
  }
  
  public void turnBackRight(double power){
    backRightDrive.setPower(power);
  }
  
  
  
  public void slideFront(double x){
    double powerSetting = .35;
    turnFrontLeft(x*powerSetting);
    turnFrontRight(-x*powerSetting);
  }
  
  public void slideRear(double x){
    double powerSetting = .55;
    turnBackLeft(x*powerSetting);
    turnBackRight(-x*powerSetting);
  }
  
  //public void slideRightTimed(double x)
  
  public void move(double x, double y, double turn){
    
        double rawTheta = Math.atan2(y,x);
        double theta = rawTheta -Math.PI/4;
        double leftFront = 0;
        double rightFront = 0;
        double leftRear = 0;
        double rightRear = 0;
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        double max = Math.max(Math.abs(sin),Math.abs(cos));
      
        double power = Math.sqrt(x*x+y*y);
        
        leftFront = power * sin/max - turn;
        rightFront = power * cos/max + turn;
        leftRear = power * cos/max - turn;
        rightRear = power * sin/max + turn;
        
        if ((power + Math.abs(turn)) > 1) {
          leftFront   /= power + Math.abs(turn);
          rightFront /= power + Math.abs(turn);
          leftRear    /= power + Math.abs(turn);
          rightRear  /= power + Math.abs(turn);
        }
        turnFrontLeft(leftFront);
        turnFrontRight(rightFront);
        turnBackLeft(leftRear);
        turnBackRight(rightRear);
    }
    
  public void allStop(){
    turnFrontLeft(0);
    turnFrontRight(0);
    turnBackLeft(0);
    turnBackRight(0);
  }
  
    public void forward(double speed){
        move(0,speed,0);
    }
    
    public void backward(double speed){
        move(0,-speed,0);
    }
    
    public void strafeLeft(double speed){
      move(-speed,0,-.04);    
    }
    
    public void strafeRight(double speed){
      move(speed,0,.04);    
    }
    
    public void forwardTimed(double speed,int time) 
    throws InterruptedException, ExecutionException
      {
        move(0,speed,0);
        Thread.sleep(time);
        allStop();
    }
    
    public void backwardTimed(double speed,int time) 
    throws InterruptedException, ExecutionException
      {
        move(0,-speed,0);
        Thread.sleep(time);
        allStop();
    }
    
    public void strafeLeftTimed(double speed,int time)
    throws InterruptedException, ExecutionException{
      strafeLeft(speed); 
      Thread.sleep(time);   
      allStop();
    }
    
    public void strafeRightTimed (double speed,int time)
    throws InterruptedException, ExecutionException
      {
        strafeRight(-speed);
        Thread.sleep(time);
        allStop();    
    }
    
    public void rotateRight(double speed){
      move(0,0,speed);    
    }
    
    public void rotateLeft(double speed){
      move(0,0,-speed);    
    }
    
    public void rotateRightTimed(double speed,int time) 
    throws InterruptedException, ExecutionException
      {
        move(0,0,speed);
        Thread.sleep(time);
        allStop();
      }
    
    public void rotateLeftTimed(double speed,int time) 
    throws InterruptedException, ExecutionException
      {
        move(0,0,-speed);
        Thread.sleep(time);
        allStop();
      }
  }
