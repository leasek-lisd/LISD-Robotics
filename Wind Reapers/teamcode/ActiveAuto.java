
package org.firstinspires.ftc.teamcode;
import java.util.concurrent.ExecutionException;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.*;//YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import java.util.List;

@Autonomous

  public class ActiveAuto extends LinearOpMode {
  private IMU imu;
  private Servo elbowServo;
  private Servo clawServo;
  private Servo elevationServo;
  private Servo lifterServo;
  private Servo launcherServo;
  private DistanceSensor distanceForward;
  private ColorSensor colorDown;
  private TouchSensor armMagnetC0;
  private int propLocation = 0;
  private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
  private AprilTagDetection desiredTag = null; 
  private VisionPortal visionPortal; 
  private AnalogInput pot1;
  private double turnSpeed = .5;  
  boolean USE_WEBCAM;
  AprilTagProcessor myAprilTagProcessor;
  VisionPortal myVisionPortal;
  DriveMotors motors = new DriveMotors(hardwareMap,imu, this); // frontLeftDrive, frontRightDrive,backLeftDrive, backRightDrive);
    
  @Override
  public void runOpMode() {
    elbowServo = hardwareMap.get(Servo.class, "elbowServoC0");
    clawServo = hardwareMap.get(Servo.class, "clawServoC1");
    lifterServo = hardwareMap.get(Servo.class, "lifterServoC3");
   // imu = hardwareMap.get(IMU.class, "imu");
    pot1 = hardwareMap.get(AnalogInput.class, "pot1");
    elevationServo = hardwareMap.get(Servo.class, "elevationServoE2");
    launcherServo = hardwareMap.get(Servo.class, "launcherServoE0");
    
      
    int tagNumber = 0;  
   
    clawServo.scaleRange(0, 1);
    lifterServo.scaleRange(0, 1);
    launcherServo.scaleRange(0, 1);
    
    distanceForward = hardwareMap.get(DistanceSensor.class, "distanceForwardC1");
    colorDown = hardwareMap.get(ColorSensor.class, "colorDownC2");
    
    initAprilTag();
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    int startPos = 0;
    
    if(pot1.getVoltage()<.5){
      startPos = 2;
    }
    else if(pot1.getVoltage()>2.7){
      startPos = 1;
    }
    else if(pot1.getVoltage()<1.5){
      startPos = 4;
    }
    else{
      startPos = 3;
    }
    
    // startPos = 4;
    telemetry.addData("Selected Start Position: ", startPos);
    telemetry.update();
    elevationServo.setPosition(.3);
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    clawServo.setPosition(.65);
    
    RoboCam camera1 = new RoboCam(myAprilTagProcessor,myVisionPortal,motors,this);
    Launcher drone = new Launcher(launcherServo, elevationServo);
    Claw claw = new Claw(clawServo);
    Arm arm = new Arm(elbowServo,hardwareMap);
         
    
    waitForStart();
    
    imu.resetYaw();
    boolean found = false;
    int blueValue = 300;
    int redValue = 300;
    double powerSetting = 1.0;
    
    motors.setStepPower(powerSetting);
    motors.forwardInches(4);
    
    double lastDistance=distanceForward.getDistance(DistanceUnit.INCH);
    
    // Allows time to receive intial distance values
    while(distanceForward.getDistance(DistanceUnit.INCH)==lastDistance){
      telemetry.addData("distance: ",distanceForward.getDistance(DistanceUnit.INCH));
      telemetry.update();
    }
      
      
      
    motors.setStepPower(powerSetting);
    tagNumber = quickPixelSearch(motors,startPos);
      telemetry.addData("tagNumber: ",tagNumber);
      telemetry.update();
    moveToLine(motors,tagNumber);
      telemetry.addData("Going to tag: ",tagNumber);
      telemetry.update();
    moveToBoard(motors,tagNumber);
      telemetry.addLine("Done");
      telemetry.update();
      sleep(25000);
      
  }
    
  public int quickPixelSearch(DriveMotors motors, int startLocation){
      if(distanceForward.getDistance(DistanceUnit.INCH)<28&&distanceForward.getDistance(DistanceUnit.INCH)>18){
        switch(startLocation){
          case 1:
            return 2;
          case 2:
            return 5;
          case 3:
            return 20;
          case 4:
            return 50;
        }
      }
      else{
        switch(startLocation){
          case 1:
          case 4:
            motors.rotateLeftDegrees(20,.5);
            
            break;
          case 2:
          case 3:
            motors.rotateRightDegrees(20,.5);
            break;
        }
      sleep(100);
          if(distanceForward.getDistance(DistanceUnit.INCH)<30&&distanceForward.getDistance(DistanceUnit.INCH)>18){
            switch(startLocation){
              case 1:
                return 1;
              case 2:
                return 6;
              case 3:
                return 30;
              case 4:
                return 40;
            } // Maybe add search if not found?
          }
          else {
            switch(startLocation){
              case 1:
                return 3;
              case 2:
                return 4;
              case 3:
                return 10;
              case 4:
                return 60;
            }
          }
        }
        return 0;
    }
    
    public void moveToLine(DriveMotors motors, int tagNumber){
      
      switch(tagNumber){
        case 1:
          
          motors.rotateRightDegrees(20,turnSpeed);
          motors.leftInches(19);
          motors.forwardInches(31);
          motors.leftInches(2);
      //    motors.rightInches(4);
          dropPixel();
          motors.leftInches(6);
          motors.rotateLeftDegrees(90,turnSpeed);
          break;
        case 40:
      /*    motors.rotateRightDegrees(20,turnSpeed);
          motors.leftInches(22);
          motors.forwardInches(34);
      //    motors.rightInches(4);
          dropPixel();
          
          motors.leftInches(4);*/
          
          motors.forwardInches(29);
          motors.rotateLeftDegrees(55,turnSpeed);
          motors.forwardInches(2);
          dropPixel();
          motors.leftInches(27);
          motors.backwardInches(65);
          motors.rightInches(28);
          motors.rotateLeftDegrees(60,turnSpeed);
          motors.rotateLeftDegrees(60,turnSpeed);
          motors.rotateLeftDegrees(60,turnSpeed);
          
          sleep(30000);
          break;
        case 2:
          motors.rotateLeftDegrees(90,.25);
          motors.backwardInches(2);
          motors.rightInches(30);
          dropPixel();
          break;
        case 50:
          motors.leftInches(2);
          motors.rotateLeftDegrees(90,.25);
       //   motors.backwardInches(2);
          motors.rightInches(31);
          dropPixel();
          motors.leftInches(4);
          //sleep(30000);
          break;
        case 3:
          motors.rotateRightDegrees(20,turnSpeed);
          motors.forwardInches(26);
          motors.rotateLeftDegrees(90,turnSpeed);
          motors.backwardInches(10);
          dropPixel();
          motors.leftInches(3);
          break;
          case 60:
          motors.rotateRightDegrees(20,turnSpeed);
          motors.forwardInches(25);
          motors.rotateLeftDegrees(90,turnSpeed);
          motors.backwardInches(10);
          dropPixel();
        //  sleep(30000);
          motors.leftInches(3);
          break;
        case 600:
          motors.rotateRightDegrees(20,turnSpeed);
          motors.forwardInches(20);
          motors.rotateRightDegrees(90,.25);
          motors.forwardInches(4);
          dropPixel();
          sleep(30000);
          break;
          
        case 4:  
          motors.rotateLeftDegrees(20,.25);
          motors.forwardInches(28);
          motors.rotateLeftDegrees(90,.25);
          motors.forwardInches(17);
          dropPixel();
          motors.leftInches(4);
       //   sleep(500);
          
          sleep(30000);
          break;
        case 10:
          motors.rotateLeftDegrees(20,.25);
          motors.forwardInches(28);
          motors.rotateLeftDegrees(90,.25);
          motors.forwardInches(17);
          dropPixel();
       //   sleep(500);
          
       //   sleep(30000);
          break;
          
        case 5:
          motors.rotateLeftDegrees(90,.25);
         // sleep(50);
          motors.backwardInches(1);
          motors.rightInches(30);
          dropPixel();
        //  sleep(10000);
        sleep(30000);
          motors.leftInches(6);
          //motors.forwardInches(18);
          break;
        case 20:
          motors.rotateLeftDegrees(90,.25);
        //  sleep(50);
          motors.backwardInches(2);
          motors.rightInches(30);
          dropPixel();
        //  sleep(10000);
          motors.leftInches(6);
          //motors.forwardInches(18);
          break;
        
        case 6:
          motors.rotateLeftDegrees(25,.25);
          motors.forwardInches(36);
          motors.rotateRightDegrees(90,.25);
          motors.forwardInches(10);
          dropPixel();
          sleep(30000);
          break;
        case 30:
          motors.rotateLeftDegrees(25,.25);
          motors.forwardInches(36);
          motors.rightInches(1.5);
       //   motors.rotateRightDegrees(90,.25);
        //  motors.forwardInches(12);
          dropPixel();
          break;
        
      }
    }
    
    public void moveToBoard(DriveMotors motors, int tagNumber){
      double turnSpeed = .75;
      switch(tagNumber){  // april tag number based on location pixel is found.  *10 for far side
        case 0:
          break;
        case 1:
          motors.forwardInches(3);
          motors.leftInches(32);
          motors.forwardInches(20);
          break;
        case 2:
          motors.leftInches(6);
          motors.forwardInches(10);
          motors.leftInches(27);
          motors.forwardInches(30);
          break;
        case 3:
          motors.forwardInches(20);
          motors.leftInches(21);
          motors.forwardInches(35);
          //dropPixel(tagNumber);
          break;
        case 4:
          motors.backwardInches(20);
          motors.rotateRightDegrees(179,turnSpeed);
        //  motors.forwardInches(36);
        //  motors.rightInches(24);
          break;
        case 5:
          motors.backwardInches(10);
         // motors.rotateRightDegrees(90,turnSpeed);
        //  motors.forwardInches(30);
          motors.leftInches(27);
          motors.backwardInches(30);
          break;
        case 6:
          motors.leftInches(6);
          motors.forwardInches(3);
          //motors.rotateRightDegrees(90,turnSpeed);
          motors.rightInches(28);
          motors.forwardInches(24);
          break;
        case 10:
          motors.leftInches(4);
     //     sleep(100);
          motors.backwardInches(18);
          motors.rightInches(28);
          motors.forward(.75);
          sleep(2200);
          motors.allStop();
          break;
       //   motors.rotateLeftDegrees(30,.25);
     //     sleep(20000);
        case 20:
          motors.backwardInches(20);
       //   sleep(100);
          motors.rightInches(31);
      //    sleep(100);
           motors.forward(.75);
          sleep(2500);
          motors.allStop();
          break;
        case 30:
     //     motors.backwardInches(18);
     //     sleep(100);
    //      motors.leftInches(20);
    //      sleep(100);
    //      motors.backward(.75);
    //      sleep(2000);
    //      motors.allStop();
          break;
          //dropPixel(tagNumber);
         
        case 40:
          motors.forwardInches(12);
          motors.rotateRightDegrees(70,turnSpeed);
          motors.forward(.75);
          sleep(1800);
          motors.allStop();
          motors.rightInches(14);
          break;
        case 50:
          motors.forwardInches(12);
          motors.rotateRightDegrees(80,turnSpeed);
          motors.forwardInches(24);
          motors.rotateRightDegrees(80,turnSpeed);
          motors.forward(.75);
          sleep(1800);
          motors.allStop();
          motors.rightInches(24);
          break;
        case 60:
          motors.forwardInches(12);
          //motors.forwardInches(24);
        //  motors.rotateRightDegrees(90,turnSpeed);
          motors.rightInches(28);
          motors.rotateRightDegrees(165,turnSpeed);
          motors.forward(.75);
          sleep(1500);
          motors.allStop();
          
          motors.rightInches(32);
          break;  
      
      }
    }
    
    
    
    
    
      public double getYaw(){
      YawPitchRollAngles orientation;
      
      orientation = imu.getRobotYawPitchRollAngles();
      return orientation.getYaw(AngleUnit.DEGREES);
    }
     
  private void dropPixel(){
    double i = 0;
    i = 1;
    lifterServo.setPosition(0);
  //  sleep(50000);
  }  
  
  
  
  private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    
private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

  if (visionPortal == null) {
    return;
  }

        // Make sure camera is streaming before we try to set the exposure controls
  if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
    telemetry.addData("Camera", "Waiting");
    telemetry.update();
    while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
      sleep(20);
    }
    telemetry.addData("Camera", "Ready");
    telemetry.update();
  }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    
    
}
