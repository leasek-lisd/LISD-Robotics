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
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor lateralEncoder;
    DcMotor forwardEncoder;
    private IMU imu;
    double stepPower = 1;//.75;
    double inchesToSteps = 44; //44;
    private LinearOpMode opMode;
    private Telemetry telemetry;
  
  public DriveTrain(HardwareMap hardwareMap, IMU newImu, LinearOpMode opmode)
    /*    DcMotor inFrontLeftDrive,
        DcMotor inFrontRightDrive,
        DcMotor inBackLeftDrive,
        DcMotor inBackRightDrive
    )*/{
      opMode = opmode;
        imu=newImu;
      frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDriveE1"); 
    frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDriveE0");
    backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDriveE3");
    backRightDrive = hardwareMap.get(DcMotor.class, "backRightDriveE2");
    lateralEncoder = hardwareMap.get(DcMotor.class, "lateralEncoderC2");
    forwardEncoder = hardwareMap.get(DcMotor.class, "forwardEncoderC3");
   
  //  frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
  //  backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
  //  frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
  //  backRightDrive.setDirection(DcMotor.Direction.REVERSE);
    
    
    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    
    }
    

  public abstract void frontLeftPower(double power);
  public abstract void frontRightPower(double power);
  public abstract void backLeftPower(double power);
  public abstract void backRightPower(double power);

  public int getFrontLeftPosition(){
    return frontLeftDrive.getCurrentPosition();
  }

  public void setStepPower(double power){
    stepPower = power;
    opMode.sleep(50);
  }

  public boolean areBusy(){
    return(
      (
      frontLeftDrive.isBusy()
      || frontRightDrive.isBusy()
      || backRightDrive.isBusy()
      || backLeftDrive.isBusy()
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
        while(areBusy()&&opMode.opModeIsActive()){};
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
        while(areBusy()&&opMode.opModeIsActive()){};
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
       while(areBusy()&&opMode.opModeIsActive()){};
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
    public void setRunToPosition(){
      frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    
    
    
    
    public void setTargetToStrafe(int stepPosition){
      frontLeftDrive.setTargetPosition(stepPosition);
      frontRightDrive.setTargetPosition(-stepPosition);
      backLeftDrive.setTargetPosition(-stepPosition);
      backRightDrive.setTargetPosition(stepPosition);
    }
    
    
    
    public void encoderModeOff(){
      frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public void rotateRightDegrees(int angle, double speed){
      encoderModeOff();
      imu.resetYaw();
    /*  if(angle>180){
        rotateLeftDegrees(angle-180,speed);
      }
      else {*/
      double startYaw=getYaw();
      while(Math.abs(getYaw() - startYaw) < angle && opMode.opModeIsActive()){
          move(0,0,speed);
         
      }
      allStop();
    //  }
    }
     
     public void rotateLeftDegrees(int angle, double speed){
      encoderModeOff();
      imu.resetYaw();
      double startYaw=getYaw();
//      if(angle>180){
//        rotateRightDegrees(angle-180,speed);
//      }
//      else {
      while(getYaw()< startYaw + angle&&opMode.opModeIsActive()){
          move(0,0,-speed);
         
      }
      allStop();
//      }
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
    
    public void forwardOdd(double inches, double power){
      
    }
    
    public int getCurrentStepLeft(){
      return frontRightDrive.getCurrentPosition();
    }
    
    public int getCurrentStepBack(){
      return frontRightDrive.getCurrentPosition();
    }
    
    public int getCurrentStepLateral(){
      return lateralEncoder.getCurrentPosition();
    }
    
    public int getCurrentStepForward(){
      return forwardEncoder.getCurrentPosition();
    }
    
    public void backwardInches2(double inches, double speed){
      double convRatio = 855.15;
      double endSteps = inches * -convRatio + getCurrentStepLeft();
      
      
      setAllPower(-speed);
      while(getCurrentStepLeft()> endSteps){
        
        
      }
      allStop();
    }
    
    public void rightDegrees(double degrees, double speed){
      double convRatio = 122;
      double endSteps = degrees * convRatio + getCurrentStepLeft();
      
      
      rotate(speed);
      while(getCurrentStepLeft()< endSteps){
        
        
      }
      allStop();
    }
    
    public double forwardInches2(double inches, double speed){
      double convRatio = 840;
      double maxPower = 0;
      double currentPower = 0;
      double endSteps = getCurrentStepLeft() - inches * convRatio;
    
     // setAllPower(getPower(endSteps,0,5,50)); //setAllPower(speed);
      while(getCurrentStepLeft() != endSteps&& opMode.opModeIsActive()){
        double error = -endSteps + getCurrentStepLeft();
        currentPower = getPropPart(error,.5);//getPower(endSteps,.25,1,10);
        maxPower = Math.max(currentPower,maxPower);
       setAllPower(currentPower); 
       
      // allStop();
      // opMode.sleep(100);
      }
      allStop();
      return maxPower;
    }
    
    public double getPropPart(double error,double p){
      return Math.max(.1,Math.min(1,p*.00025*error));
    }
    
    public void setAllPower(double speed){
      double rightCorrection = .1;
      frontLeftPower(speed);
      frontRightPower(speed*(1-rightCorrection));
      backLeftPower(speed);
      backRightPower(speed*(1-rightCorrection));
    }
    
    
    
    
    public double getPower(double setPoint, double p, int d, int interval){
      double minPower=.25;
      double lastSteps = getCurrentStepLeft();
      opMode.sleep(interval);
      double error = -setPoint + getCurrentStepLeft();
      double pMax = Math.min( ((int)((p*error/100)+.5))/100,1.0);
      double stepsMoved = -lastSteps+getCurrentStepLeft();
      double nextPos = d*(stepsMoved);
      double dMax = 1;
      
      if(nextPos<setPoint){
        dMax = error/(d*stepsMoved);
      }
      
      
      
      
      return getPropPart(error,p);//Math.max(minPower,Math.min(pMax,dMax));
      
    }
    public double leftInches2(double inches, double speed){
      double convRatio = 0;
      double maxPower = 0;
      double currentPower = 0;
      double endSteps = inches * convRatio + getCurrentStepLateral();
    
     // setAllPower(getPower(endSteps,0,5,50)); //setAllPower(speed);
      while(getCurrentStepLateral() != endSteps&& opMode.opModeIsActive()){
        double error = endSteps - getCurrentStepLeft();
        currentPower = getPropPart(error,.5);//getPower(endSteps,.25,1,10);
        maxPower = Math.max(currentPower,maxPower);
       setAllPowerStrafe(currentPower); 
       
      // allStop();
      // opMode.sleep(100);
      }
      allStop();
      return maxPower;
    }
    
    public void setAllPowerStrafe(double speed){
      double rightCorrection = .1;
      frontLeftPower(speed);
      frontRightPower(-speed*(1-rightCorrection));
      backLeftPower(-speed);
      backRightPower(speed*(1-rightCorrection));
    }
}