package org.firstinspires.ftc.teamcode;




    // todo: write your code here
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.ExecutionException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.*;//YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class DriveTrain {
    //Telemetry telemetry = opMode.telemetry;
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor backLeft;
    DcMotor backRight;
    private IMU imu;
    double stepPower = .5;
    double inchesToSteps = 44;
    private LinearOpMode opMode;
    private Telemetry telemetry;
    
  public DriveTrain(HardwareMap hardwareMap, IMU newImu, LinearOpMode opmode)
    /*    DcMotor inFrontLeft,
        DcMotor inFrontRight,
        DcMotor inbackLeft,
        DcMotor inbackRight
    )*/{
      opMode = opmode;
        imu=newImu;
    FrontLeft = hardwareMap.get(DcMotor.class, "frontLeftE1"); 
    FrontRight = hardwareMap.get(DcMotor.class, "frontRightC2");
    backLeft = hardwareMap.get(DcMotor.class, "backLeftE2");
    backRight = hardwareMap.get(DcMotor.class, "backRightC1");
   
    FrontLeft.setDirection(DcMotor.Direction.REVERSE);
    backLeft.setDirection(DcMotor.Direction.REVERSE);
    //FrontRight.setDirection(DcMotor.Direction.REVERSE);
    //backRight.setDirection(DcMotor.Direction.REVERSE);
    
    
    FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    
    }
    

  public abstract void frontLeftPower(double power);
  public abstract void frontRightPower(double power);
  public abstract void backLeftPower(double power);
  public abstract void backRightPower(double power);

  public int getFrontLeftPosition(){
    return FrontLeft.getCurrentPosition();
  }

  public void setStepPower(double power){
    stepPower = power;
  }

  public boolean areBusy(){
    return(
      (
      FrontLeft.isBusy()
      || FrontRight.isBusy()
      || backRight.isBusy()
      || backLeft.isBusy()
      ));
  }
  public void slideFront(double x){
    encoderModeOff();
    double powerSetting = .35; // tune these for your bot
    frontLeftPower(x*powerSetting);
    frontRightPower(-x*powerSetting);
  }
  
  public void slideRear(double x){ // tune these for your bot
    encoderModeOff();
    double powerSetting = .55;
    
    backLeftPower(-x*powerSetting);
    backRightPower(x*powerSetting);
  }
  
  //public void slideRightTimed(double x)
  public void move2(double x, double y, double turn,double offset){
    move3(x,y,turn,offset);
  }
  public void move(double x, double y, double turn){
    move3(x,y,turn,0);
  }
  public void move3(double x, double y, double turn,double offset){
    
        double rawTheta = Math.atan2(y,x);
        double theta = rawTheta - 1*Math.PI/4+2*offset;
        
        double leftFront = 0;
        double rightFront = 0;
        double leftRear = 0;
        double rightRear = 0;
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        double max = Math.max(Math.abs(sin),Math.abs(cos));
      
        double power = - Math.sqrt(x*x+y*y);
        
        leftFront = power * sin/max + turn;
        rightFront = power * cos/max - turn;
        leftRear = power * cos/max + turn;
        rightRear = power * sin/max - turn;
        
        if ((power + Math.abs(turn)) > 1) {
          leftFront   /= power + Math.abs(turn);
          rightFront /= power + Math.abs(turn);
          leftRear    /= power + Math.abs(turn);
          rightRear  /= power + Math.abs(turn);
        }
        frontLeftPower(leftFront);
        frontRightPower(rightFront);
        backLeftPower(leftRear);
        backRightPower(rightRear);
    }
    
  public void allStop(){
    frontLeftPower(0);
    frontRightPower(0);
    backLeftPower(0);
    backRightPower(0);
  }
  
    public void forward(double speed){
      encoderModeOff();
        move(0,-speed,0);
    }
    
    public void backward(double speed){
      encoderModeOff();
        move(0,speed,0);
    }
    
    public void strafeLeft(double speed){
      encoderModeOff();
      move(-speed,0,0);    
    }
    
    public void strafeRight(double speed){
      encoderModeOff();
      move(speed,0,0);    
    }
    
    public void forwardInches(double inches){
      forwardInches(inches,true);
    }
    
    public void forwardInches(double inches,boolean wait){
      int steps = (int) (inches * inchesToSteps);
      forwardSteps(steps,stepPower);
      if(wait){
        while(areBusy()){};
      }
    }
    
    public void backwardInches(double inches){
      backwardInches(inches, true);
    }
    public void backwardInches(double inches,boolean wait){
      int steps = (int) (inches * inchesToSteps);
      forwardInches(-inches);
    }
    
    public void leftInches(double inches){
      leftInches(inches,true);
    }
    
    public void leftInches(double inches,boolean wait){
      int steps = (int) (inches * inchesToSteps);
      leftSteps(-steps,stepPower);
      if(wait){
        while(areBusy()){};
      }
    }
    
    
    public void rightInches(double inches){
      rightInches(inches,true);
    }
    public void rightInches(double inches,boolean wait){
      int steps = (int) (inches * inchesToSteps);
      leftSteps(steps,stepPower);
      if(wait){
        while(areBusy()){};
       }
      
    }
     public void finish(){
       while(areBusy()){};
     }
    
    
    public void forwardSteps(int motorSteps, double speed){
     
      resetAllEncoders();
      setTargetTo(motorSteps);
      setRunToPosition();
      setAllPower(speed);
      //while(areBusy()){};
      
      
    }
    
    public void leftSteps(int motorSteps, double speed){
      resetAllEncoders();
      setTargetToStrafe(motorSteps);
      setRunToPosition();
      setAllPower(speed);
  //    while(areBusy()){};
  //    encoderModeOff();
    }  
    
    public void resetAllEncoders(){
      FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setTargetTo(int stepPosition){
      FrontLeft.setTargetPosition(stepPosition);
      FrontRight.setTargetPosition(stepPosition);
      backLeft.setTargetPosition(stepPosition);
      backRight.setTargetPosition(stepPosition);
    }
    public void setRunToPosition(){
      FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    
    
    
    
    public void setTargetToStrafe(int stepPosition){
      FrontLeft.setTargetPosition(stepPosition);
      FrontRight.setTargetPosition(-stepPosition);
      backLeft.setTargetPosition(-stepPosition);
      backRight.setTargetPosition(stepPosition);
    }
    
    public void setAllPower(double speed){
      frontLeftPower(speed);
      frontRightPower(speed);
      backLeftPower(speed);
      backRightPower(speed);
    }
    
    public void encoderModeOff(){
      FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public void rotateRightDegrees(int angle, double speed){
      imu.resetYaw();
      encoderModeOff();
     
      double startYaw=getYaw();
      while(Math.abs(getYaw() - startYaw) < angle && opMode.opModeIsActive()){
          move(0,0,speed);
         
      }
      allStop();
      
    }
     
     public void rotateLeftDegrees(int angle, double speed){
      imu.resetYaw();
      double startYaw=getYaw();
     
      while(getYaw()< startYaw + angle){
          move(0,0,-speed);
         
      }
      allStop();
      
    } 
    
    public void rotate(double speed){
      encoderModeOff();
      move(0,0,speed); 
    }
    
    public void rotateRight(double speed){
      rotate(speed);
    }
    
    public void rotateLeft(double speed){
      rotate(-speed);   
    }
    public double getYaw(){
      YawPitchRollAngles orientation;
      
      orientation = imu.getRobotYawPitchRollAngles();
      return orientation.getYaw(AngleUnit.DEGREES);
    }
    
    
}
