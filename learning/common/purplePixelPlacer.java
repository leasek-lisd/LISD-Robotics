package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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


@Autonomous(name = "Purple Pixel Placer Test")
public class BaseTemplate extends LinearOpMode {

  private DcMotor frontLeftDrive;
  private DcMotor frontRightDrive;
  private DcMotor backLeftDrive;
  private DcMotor backRightDrive;
  private Servo servo0;
  private Servo servo1;
  private DistanceSensor distance1;
  private ColorSensor color;

  
  boolean USE_WEBCAM;
  AprilTagProcessor myAprilTagProcessor;
  double servoPos;
  VisionPortal myVisionPortal;
  double servo0Up = .5;
  double servo0Down = .2;
  double legLength = 10.0;
  double maxPropRange = 10.0;
  double minPropRange = 8.0;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    
    TestClass robot1 = new TestClass();
    
    frontLeftDrive = hardwareMap.get(DcMotor.class, "motor0"); 
    frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    frontRightDrive = hardwareMap.get(DcMotor.class, "motor1");
    backLeftDrive = hardwareMap.get(DcMotor.class, "motor2");
    backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    backRightDrive = hardwareMap.get(DcMotor.class, "motor3");
    servo0 = hardwareMap.get(Servo.class, "servo0");
    
    servo1 = hardwareMap.get(Servo.class, "servo1");
    distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
    color = hardwareMap.get(ColorSensor.class, "color");

    // Put initialization blocks here.
    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    USE_WEBCAM = true;
    servo0.scaleRange(0, 1);
    initAprilTag();
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch Play to start OpMode");
    telemetry.update();
    servo1.setPosition(servo0Down);
    waitForStart();
    if (opModeIsActive()) {
      int program = 1;
      
      if(program == 1){
      
      forwardTimed(500,.5);
      sleep(500);
      strafeRightTimed(500,.5);
      sleep(500);
      backwardTimed(500,.5);
      sleep(500);
      strafeLeftTimed(500,.5);
      sleep(500);
      rotateLeft(500,.5);
      sleep(500);
      rotateRight(1000,.5);
      }
      else{
      if(findPropBehind(1)){
        //next auto method
      }
      }
    }
    
  
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
       
       
       boolean atFirstLeg = false;
       boolean pastFirstLeg = false;
       boolean propFound = false;
       boolean skipLine = false;
       boolean firstLineFound = true;
       boolean colorMatch = false;
       
       
       while(!(atFirstLeg&&pastFirstLeg&propFound)) {
         pingDistance = distance1.getDistance(DistanceUnit.INCH);
       // accuracy can be improved with april tags but would require camera facing that direction 
       // also need to add something from a timeout in case it gets lost
         if(!atFirstLeg&&pingDistance <= legLength){
           atFirstLeg = true;
         }
         if(atFirstLeg&&pingDistance > legLength){
           pastFirstLeg = true;
         }
         if(atFirstLeg&&pastFirstLeg&&pingDistance <= maxPropRange){
           propFound = true;
           if(pingDistance>minPropRange){
             firstLineFound = false;
             skipLine = true;
           }
         }
       }
       allStop();
       sleep(100);
       
       // Add something to verify stop in correct position
       propFound = false;
       strafeLeft(.5);
       while(!propFound){
         // need to add timeout
         // need to add "lost" logic
         
         colorMatch = 400 < color.blue() || 400 < color.red();
         
         if(skipLine&&colorMatch){
           firstLineFound = true;
         }
         
         if(skipLine&&firstLineFound&&!colorMatch){
           skipLine = false;
         }
         
         if(!skipLine&&firstLineFound&&colorMatch){
           propFound = true;
         }
         
       }
       allStop();
       //raise servo arm
       servo1.setPosition(servo0Up);
       strafeRightTimed(100,.5);
       
     }
    
       
       
       return true;
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
    
    public void rotate(int runTime, double speed){
      frontLeftDrive.setPower(-speed);
      frontRightDrive.setPower(speed);
      backLeftDrive.setPower(-speed);
      backRightDrive.setPower(speed);
      sleep(runTime);
      allStop();
    }
    
    public void rotateRight(int runTime,double speed){
      rotate(runTime, speed);
    }
    public void rotateLeft(int runTime,double speed){
      rotate(runTime, -speed);
    }
    
    public void strafeRight(double speed){
    frontLeftDrive.setPower(speed);    
      frontRightDrive.setPower(-speed);
      backLeftDrive.setPower(-speed);
      backRightDrive.setPower(speed);
  }
  
  public void strafeLeft(double speed){
    frontLeftDrive.setPower(-speed);
      frontRightDrive.setPower(speed);
      backLeftDrive.setPower(speed);
      backRightDrive.setPower(-speed);
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
      myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      // Use the device's back camera.
      myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    // Add myAprilTagProcessor to the VisionPortal.Builder.
    myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
    // Create a VisionPortal by calling build.
    myVisionPortal = myVisionPortalBuilder.build();
  }
  
}
