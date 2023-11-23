package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "ReaperTime")
public class TeleReaper extends LinearOpMode {

  private DcMotor frontLeftDrive;
  private DcMotor frontRightDrive;
  private DcMotor backLeftDrive;
  private DcMotor backRightDrive;
  private DcMotor leftArmLift;
  private DcMotor rightArmLift;
  private Servo servo0;
  private Servo servo1;
 // private Servo servo2;
  private Servo servo3;
  private DistanceSensor distance1;
  private ColorSensor color;

  
  boolean USE_WEBCAM;
  AprilTagProcessor myAprilTagProcessor;
  double servoPos;
  double servoPos2;
  VisionPortal myVisionPortal;
  double servo3Up = 0.45;
  double servo3Down = 0.32;
  

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    
 //   TestClass robot1 = new TestClass();
    
    frontLeftDrive = hardwareMap.get(DcMotor.class, "motor0"); 
    frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    frontRightDrive = hardwareMap.get(DcMotor.class, "motor1");
    frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
    backLeftDrive = hardwareMap.get(DcMotor.class, "motor2");
    backLeftDrive.setDirection(DcMotor.Direction.REVERSE); 
    backRightDrive = hardwareMap.get(DcMotor.class, "motor3");
    backRightDrive.setDirection(DcMotor.Direction.REVERSE);
    leftArmLift = hardwareMap.get(DcMotor.class, "motor4");
    rightArmLift = hardwareMap.get(DcMotor.class, "motor5");
    servo0 = hardwareMap.get(Servo.class, "servo0");
    servo1 = hardwareMap.get(Servo.class, "servo1");
 //   servo2 = hardwareMap.get(Servo.class, "servo2");
    servo3 = hardwareMap.get(Servo.class, "servo3");
    
    
    distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
    color = hardwareMap.get(ColorSensor.class, "color");

    // Put initialization blocks here.
    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    USE_WEBCAM = true;
    servo0.scaleRange(0, 1);
    servo1.scaleRange(0, 1);
  //  servo2.scaleRange(0, 1);
    servo3.scaleRange(0, 1);
    initAprilTag();
    
  //  servo0.setPosition(servo0Down);
    
    waitForStart();
 //    servoPos = 0;
 //    servoPos2 = 0;
 //   servo3.setDirection(Servo.Direction.FORWARD);
   servo0.setPosition(0);
    servo1.setPosition(.2);
 //   servo2.setPosition(0);
    servo3.setPosition(0.3);
    if (opModeIsActive()) {
      // Put run blocks here.
      
      
      while (opModeIsActive()) {
          if(gamepad1.dpad_left) {
          strafeLeftTimed(500,.7);
          //sleep(500);
          //allStop();
        }
        
        if(gamepad1.dpad_right) {
          strafeRightTimed(500,.7);
          //sleep(500);
          //allStop();
        }
        
        if(gamepad1.dpad_up) {
          forwardTimed(500,.7);
          //sleep(500);
          //allStop();
        }
        
        if(gamepad1.dpad_down) {
          forwardTimed(500,-.7);
          //sleep(500);
          //allStop();
        }
          // Movement
           omni(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x);
         // forward(-gamepad1.left_stick_y);
         //rotate(-gamepad1.right_stick_x);
          
          
          // Upper Arm
          if(Math.abs(gamepad2.left_stick_y)>.05){
            moveArm(gamepad2.left_stick_y);
          }
          else {
              stopArm();
          }
          
          // Lower Arm
          
          if (gamepad2.right_stick_y!=0){
              double adj = gamepad2.right_stick_y * .05;
            servoPos = Math.min(Math.max(servoPos + adj, 0), 1);
          //  servoPos2 = Math.min(Math.max(servoPos - 0.03, 1), 0);
            telemetry.addLine(""+servoPos+" / "+gamepad2.right_stick_y);
            telemetry.update();
        }
          
        // Claw
        if (gamepad2.left_bumper) {
         // servoPos = Math.min(Math.max(servoPos + 0.01, 0), 1);
          servoPos2 = 0; // Math.min(Math.max(servoPos + 0.03, 0), 1);
          telemetry.addLine("Up");
          telemetry.update();
        }
        telemetry.update();
        if (gamepad2.left_trigger>.05) {
            double adj = gamepad2.left_trigger * .15;
         // servoPos = Math.min(Math.max(servoPos - 0.01, 0), 1);
          servoPos2 =  Math.min(Math.max(servoPos2 + adj, 0), 1);
          telemetry.addLine("Right Stick: \n"+gamepad2.right_stick_x+","+gamepad2.right_stick_y);
          telemetry.update();
        }
        servo1.setPosition(servoPos2);
        servo0.setPosition(servoPos);
   //     servo2.setPosition(servoPos);
   //     servo3.setPosition(servoPos + 0.3);
        telemetry.update();
        sleep(50);
      }
    }
      /*if(program == 1){
      
      forwardTimed(5000,.5);
    //  sleep(5000);
     // strafeRightTimed(5000,.75);
      sleep(500);
     // backwardTimed(500,.5);
      sleep(500);
    //  strafeLeftTimed(500,.5);
      sleep(500);
  //    rotateLeft(500,.5);
      sleep(500);
    //  rotateRight(1000,.5);
    //  }
  //    else{
      // if(findPropBehind(1)){
        //next auto method
     // } 
     
     sleep(5000);
      servo0.setPosition(servo0Up);
     telemetry.addData(">", "Other Servo");
    telemetry.update();
    
    telemetry.addData(">", "Sleeping");
    telemetry.update();
   // sleep(5000);
  //    findPropBehind(1);
  */
    }
    
  
  public void moveArm(double y){
      double reduction = .5;
      leftArmLift.setPower(reduction*y); 
      rightArmLift.setPower(reduction*y); 
  }
  
  public void stopArm(){
      
      leftArmLift.setPower(0); 
      rightArmLift.setPower(0); 
  }
  
   public boolean findPropBehind(int startPos){
     /* startPos:
       1 - Blue / Nearest Backstage
       2 - Blue / Nearest Audience 
       3 - Red / Nearest Backstage
       4 - Red / Nearest Audience 
       
       Assuming left side mount
     */
     
     if(startPos==1||startPos ==4){
       strafeRightTimed(200,1);
       backward(.5);
       double pingDistance = 1;
       
       double legLength = 20.0;
       double maxPropRange = 25.0;
       double minPropRange = 2.0;
       
       boolean atFirstLeg = false;
       boolean pastFirstLeg = false;
       boolean propFound = false;
       boolean skipLine = false;
       boolean firstLineFound = true;
       boolean colorMatch = false;
       servo3.setPosition(servo3Down);
       String stateMsg = "Searching for "+legLength;
       while(!(atFirstLeg&&pastFirstLeg&propFound)) {
         
         pingDistance = distance1.getDistance(DistanceUnit.INCH);
         telemetry.addData(">", (stateMsg + " / " + pingDistance));
         telemetry.update();
       // accuracy can be improved with april tags but would require camera facing that direction 
       // also need to add something from a timeout in case it gets lost
         if(!atFirstLeg&&pingDistance <= legLength){
           atFirstLeg = true;
           stateMsg+="\nfirst leg";
         }
         if(!pastFirstLeg&&atFirstLeg&&pingDistance > legLength){
           pastFirstLeg = true;
           stateMsg+="\npast first leg";
         }
         if(atFirstLeg&&pastFirstLeg&&pingDistance <= maxPropRange){
           propFound = true;
           stateMsg+="\nfound prop";
           if(pingDistance>minPropRange){
             firstLineFound = false;
             skipLine = true;
             stateMsg+="\npast found long";
           }
           else {
             stateMsg+="\nfound short";
           }
         }
       }
       allStop();
       telemetry.addData(">", "Stopped, prop found skipLine = "+skipLine);
         telemetry.update();
       sleep(1000);
       
       // Add something to verify stop in correct position
       propFound = false;
       strafeLeft(.5);
       stateMsg = "searching for color";
       while(!propFound){
         // need to add timeout
         // need to add "lost" logic
         
         colorMatch = 400 < color.blue() || 400 < color.red();
         
         if(skipLine&&colorMatch){
           skipLine = false;
           firstLineFound = true;
           stateMsg += "found first line\n";
         }
         
         if(!skipLine&&firstLineFound&&!colorMatch){
           skipLine = false;
          // stateMsg += "passed first line\n";
           telemetry.addData(">", "passed first line");
           telemetry.update();
         }
         
         if(!skipLine&&firstLineFound&&colorMatch){
           propFound = true;
           stateMsg += "found line\n";
         }
         telemetry.addData(">", stateMsg);
         telemetry.update();
       }
       allStop();
       //raise servo arm
       sleep(250);
       servo3.setPosition(servo3Up);
       strafeRightTimed(100,.5);
        telemetry.addData(">", "Sleeping" + skipLine);
         telemetry.update();
       sleep(5000);
       
     }
    
       return true;
   }
 
 public void omni(double x, double y, double turn){
    
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
        
        leftFront = power * cos/max + turn;
        rightFront = power * sin/max - turn;
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
    
    public static double roundMe(double num){
        return Math.round(num*100)/100.0;
    }
    
 
 
 
  public void forward(double speed){
    frontLeftDrive.setPower(speed);
      frontRightDrive.setPower(speed);
      backLeftDrive.setPower(speed);
      backRightDrive.setPower(speed);
  }
  
  public void allStop(){
    frontLeftDrive.setPower(0);
      frontRightDrive.setPower(0);
      backLeftDrive.setPower(0);
      backRightDrive.setPower(0);
  }
  
  public void forwardTimed(int runTime, double speed){
      forward(speed);
      sleep(runTime);
      allStop();
    }
    
    public void backward(double speed){
      forward(-speed);
    }
    
    public void backwardTimed(int runTime, double speed){
      backward(speed);
      sleep(runTime);
      allStop();
    }
    
    public void rotate(double speed){
      frontLeftDrive.setPower(-speed);
      frontRightDrive.setPower(speed);
      backLeftDrive.setPower(speed);
      backRightDrive.setPower(-speed);
      
    }
    
    public void rotateTimed(int runTime, double speed){
      frontLeftDrive.setPower(-speed);
      frontRightDrive.setPower(speed);
      backLeftDrive.setPower(-speed);
      backRightDrive.setPower(-speed);
      sleep(runTime);
      allStop();
    }
    
    public void rotateRight(int runTime,double speed){
      rotateTimed(runTime, speed);
    }
    public void rotateLeft(int runTime,double speed){
      rotateTimed(runTime, -speed);
    }
    
    public void strafeRight(double speed){
    frontLeftDrive.setPower(speed);    
      frontRightDrive.setPower(-speed);
      backLeftDrive.setPower(speed);
      backRightDrive.setPower(-speed);
  }
  
  public void strafeLeft(double speed){
    frontLeftDrive.setPower(-speed);
      frontRightDrive.setPower(speed);
      backLeftDrive.setPower(-speed);
      backRightDrive.setPower(speed);
  }
  
  public void strafeRightTimed(int runTime,double speed){
    strafeRight(speed);
    sleep(runTime);
    allStop();
  }
  
  public void strafeLeftTimed(int runTime,double speed){
    strafeLeft(speed);
    sleep(runTime);
    allStop();
  }
  
   private void initAprilTag() {
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;

    // First, create an AprilTagProcessor.Builder.
    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    // Create an AprilTagProcessor by calling build.
    myAprilTagProcessor = myAprilTagProcessorBuilder.build();
    // Next, create a VisionPortal.Builder and set attributes related to the camera.
    myVisionPortalBuilder = new VisionPortal.Builder();
    if (USE_WEBCAM) {
      // Use a webcam.
   //   myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      // Use the device's back camera.
  //    myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    // Add myAprilTagProcessor to the VisionPortal.Builder.
//    myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
    // Create a VisionPortal by calling build.
 //   myVisionPortal = myVisionPortalBuilder.build();
  }
  
}
