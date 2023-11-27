package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.ExecutionException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class DriveMotors   {
    
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

  public boolean areBusy(){
    return((
      frontLeftDrive.isBusy()
      ||frontRightDrive.isBusy()
      ||backRightDrive.isBusy()
      ||backLeftDrive.isBusy()
      ));
  }
  public void slideFront(double x){
    double powerSetting = .35;
    backLeftDrive.setPower(x*powerSetting);
    backRightDrive.setPower(-x*powerSetting);
  }
  
  public void slideRear(double x){
    double powerSetting = .55;
    frontLeftDrive.setPower(x*powerSetting);
    frontRightDrive.setPower(-x*powerSetting);
  }
  
  //public void slideRightTimed(double x)
  public void move2(double x, double y, double turn,double offset){
    move3(x,y,turn,offset);
  }
  public void move(double x, double y, double turn){
    move3(x,y,turn,0);
  }
  public void move3(double x, double y, double turn,double offset){
    encoderModeOff();
        double rawTheta = Math.atan2(y,x);
        double theta = rawTheta -Math.PI/4+2*offset;
        
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
        frontLeftDrive.setPower(leftFront);
        frontRightDrive.setPower(rightFront);
        backLeftDrive.setPower(leftRear);
        backRightDrive.setPower(rightRear);
    }
    
  public void allStop(){
    frontLeftDrive.setPower(0);
    frontRightDrive.setPower(0);
    backLeftDrive.setPower(0);
    backRightDrive.setPower(0);
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
    
    public void forwardInches(double inches){
      //int steps = (int) (inches * 36.6735);
      int steps = (int) (inches * 44);
      forwardSteps(steps,.5);
    }
    public void backInches(double inches){
      //int steps = (int) (inches * 36.6735);
      int steps = (int) (inches * 44);
      forwardSteps(-steps,.5);
      while(frontLeftDrive.isBusy()){};
    }
    
    public void forwardSteps(int motorSteps, double speed){
     /*
      frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontLeftDrive.setTargetPosition(-motorSteps);
      frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontLeftDrive.setPower(speed);
      
      
      frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontRightDrive.setTargetPosition(-motorSteps);
      frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontRightDrive.setPower(speed);
      
      
      backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backLeftDrive.setTargetPosition(-motorSteps);
      backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backLeftDrive.setPower(speed);
      
      
      backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backRightDrive.setTargetPosition(-motorSteps);
      backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backRightDrive.setPower(speed);
      */
      resetAllEncoders();
      setTargetTo(-motorSteps);
      setRunToPosition();
      setAllPower(speed);
      // encoderModeOn();
      
     // while(frontLeftDrive.isBusy()){}
     // allStop();
     // while(opModeIsActive&&)
    }
    
    
    
    public void setRunToPosition(){
      frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void resetAllEncoders(){
      frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    public void setTargetTo(int stepPosition){
      frontLeftDrive.setTargetPosition(stepPosition);
      frontRightDrive.setTargetPosition(stepPosition);
      backLeftDrive.setTargetPosition(stepPosition);
      backRightDrive.setTargetPosition(stepPosition);
    }
    
    public void setAllPower(double speed){
      frontLeftDrive.setPower(speed);
      frontRightDrive.setPower(speed);
      backLeftDrive.setPower(speed);
      backRightDrive.setPower(speed);
    }
    
    public void encoderModeOff(){
      frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void leftInches(double inches){
      //int steps = (int) (inches * 36.6735);
      int steps = (int) (inches * 44);
      leftSteps(-steps,.5);
    }
    
    public void rightInches(double inches){
      //int steps = (int) (inches * 36.6735);
      int steps = (int) (inches * 44);
      leftSteps(steps,.5);
    }
    public void leftSteps(int motorSteps, double speed){
      frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontLeftDrive.setTargetPosition(-motorSteps);
      frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontLeftDrive.setPower(speed);
      frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      
      
      frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontRightDrive.setTargetPosition(motorSteps);
      frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontRightDrive.setPower(speed);
      frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      
      backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backLeftDrive.setTargetPosition(motorSteps);
      backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backLeftDrive.setPower(speed);
      backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      
      backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backRightDrive.setTargetPosition(-motorSteps);
      backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backRightDrive.setPower(speed);
      backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      
     // while(frontLeftDrive.isBusy()){}
     // allStop();
     // while(opModeIsActive&&)
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
