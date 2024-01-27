package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.*;//AngularVelocity;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.ExecutionException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.robotcore.external.navigation.*;

// This is the name that will appear in the Driver Hub Menu
@TeleOp(name = "Driver Template")
public class TeleOps extends LinearOpMode {


  private IMU imu;
 
  
  private Servo elbowServo;
  private Servo clawServo;
  private Servo elevationServo;
  private Servo lifterServo;
  private Servo launcherServo;
  
  private DistanceSensor distanceForward;
  private ColorSensor colorDown;
  private TouchSensor armMagnetC0;
  

  
  boolean USE_WEBCAM;
  AprilTagProcessor myAprilTagProcessor;
  VisionPortal myVisionPortal;
  YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;
    
 

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    
    

    
/* Servo Config  *********************************************  Servo Config */
    
    // Match to config file
    elbowServo = hardwareMap.get(Servo.class, "elbowServoC0");
    clawServo = hardwareMap.get(Servo.class, "clawServoC1");
    lifterServo = hardwareMap.get(Servo.class, "lifterServoC3");
    
    elevationServo = hardwareMap.get(Servo.class, "elevationServoE2");
    launcherServo = hardwareMap.get(Servo.class, "launcherServoE0");
    imu = hardwareMap.get(IMU.class, "imu");
    
    // Set Ranges
    elbowServo.scaleRange(0, 1);
    clawServo.scaleRange(0, 1);
    //extenderServo.scaleRange(0, 1);
    elevationServo.scaleRange(0, 1);
    launcherServo.scaleRange(0, 1);
    
   // Set Directions
    elbowServo.setDirection(Servo.Direction.REVERSE);
    
    // Set INIT position
    double elbowServoPos = 0;
   
   
    
   elbowServo.setPosition(.15);//retract
    double clawServoPos = 1.0;
    clawServo.setPosition(1);//open
    
    //sleep(2000);
  //  extenderServo.setPosition(1.0);
    lifterServo.setPosition(1.0);
    launcherServo.setPosition(1.0);
    elevationServo.setPosition(0);
    
/* Sensor Config  *****************************************  Sensor Config */    
    
    distanceForward = hardwareMap.get(DistanceSensor.class, "distanceForwardC1");
    colorDown = hardwareMap.get(ColorSensor.class, "colorDownC2");
 //   armMagnetC0 = hardwareMap.get(TouchSensor.class, "armMagnetC0");    
/* Webcam Config androidTextToSpeech.initialize(); *********************************************  Webcam Config */    
    
    USE_WEBCAM = true;
   // initAprilTag();
    
/* Init Telemetry  *****************************************  Init Telemetry */
   initAprilTag();
   
    waitForStart();
    lifterServo.setPosition(0);
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
    
    if (opModeIsActive()) { // *****************************************  Start
      
      
      
      clawServoPos = .6;
      clawServo.setPosition(clawServoPos);
     DriveMotors motors = new DriveMotors(hardwareMap,imu,this); // frontLeftDrive, frontRightDrive,backLeftDrive, backRightDrive);
        Launcher drone = new Launcher(launcherServo, elevationServo);
    //    GreatArm arm = new GreatArm(hardwareMap);
        
     //   Arm arm = new Arm(armMotorLeft, armMotorRight,elbowServo,armMagnetC0);
     Arm arm = new Arm(elbowServo,hardwareMap);
        Claw claw = new Claw(clawServo);
        RoboCam camera1 = new RoboCam(myAprilTagProcessor,myVisionPortal,motors,this);
    //    PurplePixelPlacer ppp = new PurplePixelPlacer();
       // try{
        //  ppp.findPropBehind(1, motors, lifterServo, colorDown, distanceForward, this);}
      //  catch(InterruptedException e){}
       // catch(ExecutionException e){};
      while (opModeIsActive()) { // **********************************    Loop
       // SticksB.y(armMotorLeft, armMotorRight,gamepad2.left_stick_y);
        double yaw = 0;
        
       // if(gamepad1.left_trigger<.1&&(gamepad1.right_trigger>.9)){
        //  yaw = getYaw(imu);
        //      }
          
        motors.move2(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,yaw);
     //   arm.setLowerArmPower(gamepad2.left_stick_y);
    //    arm.setUpperArmPower(gamepad2.right_stick_y);
        if (gamepad2.left_trigger>.9&&gamepad2.right_trigger>.9&&gamepad1.left_trigger>.9&&gamepad1.right_trigger>.9){
         drone.shoot();
        }
        if(Math.abs(gamepad2.left_stick_y)>.1){
          arm.move(gamepad2.left_stick_y);
        }
        else {
          arm.move(0);
        }
        
      
        
        if (Math.abs(gamepad2.right_stick_y)>.05){
              arm.moveElbow(gamepad2.right_stick_y);
        }
        
        if (gamepad2.left_bumper){
          claw.openClaw();
        }
        if (gamepad1.back){
          motors.frontLeftPower(.25);
          sleep(500);
          motors.allStop();
        }
        
        
        
        if (gamepad2.right_bumper){
          claw.closeClaw();
        }
        
        if (gamepad1.right_bumper){
          drone.lift();
        }
        
        if (gamepad1.right_trigger>.5&&gamepad1.left_trigger>.5){
          drone.shoot();
        }
        
        if (gamepad2.left_stick_button&&!gamepad2.right_stick_button){
          motors.move(0,.25,0);
          
        }
        if (gamepad2.right_stick_button&&!gamepad2.left_stick_button){
          motors.move(0,-.25,0);
        }
        
       
        
        if (Math.abs(gamepad2.left_stick_x)>.25){
           motors.slideFront(gamepad2.left_stick_x);
           gamepad1.rumble(100);}
        
        if (Math.abs(gamepad2.right_stick_x)>.25){
           motors.slideRear(gamepad2.right_stick_x);
           gamepad1.rumble(100);}
        
        if(gamepad1.a){
    //      try{
    //         camera1.lineUp(5,this);
    //      }
    //   catch(InterruptedException e){}
    //   catch(ExecutionException e){};
          
        }
        
        if(gamepad1.b){
          try{
             camera1.squareUp(5,this);
          }
       catch(InterruptedException e){}
       catch(ExecutionException e){};
          
        }
        //servo0.setPosition(servoPos);
      if(gamepad1.dpad_up){
        motors.frontLeftPower(.5);
      } 
      
      if(gamepad1.dpad_right){
        motors.frontRightPower(.5);
      }
      
      if(gamepad1.dpad_down){
        motors.backRightPower(.5);
      }
      
      if(gamepad1.dpad_left){
        motors.backLeftPower(.5);
      }
      
      double armSpeed = 1;
      if(gamepad2.dpad_up){
        arm.moveSteps(100,armSpeed);
      }
      
      if(gamepad2.dpad_down){
        arm.moveSteps(0,armSpeed);
      }
      
      if(gamepad2.dpad_left){
       arm.moveSteps(20,armSpeed);
      }
      
      if(gamepad2.dpad_right){
        arm.moveSteps(30,armSpeed);
      }
      
      if(gamepad2.left_trigger> .05){
       motors.move2(-gamepad2.left_trigger *.5,0,0,0);
       gamepad1.rumble(100);
      }
       
       if(gamepad2.right_trigger> .05){
       motors.move2(gamepad2.right_trigger*.5,0,0,0);
       gamepad1.rumble(100);
       }
      
      //telemetry.addData("Yaw (Z)",getYaw(imu));
      
      //telemetry.update();
      
//      try{
          //  telemetry.addData("x",camera1.getTagLocation(5));
  //          telemetry.update();
  //        }
  //     catch(InterruptedException e){}
  //     catch(ExecutionException e){};
      }
    }
      
    }
    
    
    private double getYaw(IMU imu){
      YawPitchRollAngles orientation;
      orientation = imu.getRobotYawPitchRollAngles();
      return orientation.getYaw(AngleUnit.RADIANS);
    }
    
  private void initAprilTag() {
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;

    // First, create an AprilTagProcessor.Builder.
    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    myAprilTagProcessorBuilder.setLensIntrinsics(835.112, 835.112, 322.08, 242.678);
    
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

  /**
   * Display info (using telemetry) for a recognized AprilTag.
   */
  
  
}