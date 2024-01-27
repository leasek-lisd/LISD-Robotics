// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
// import org.firstinspires.ftc.vision.tfod.TfodProcessor;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
// import com.qualcomm.robotcore.hardware.TouchSensor;
// import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import java.util.concurrent.ExecutionException;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// 
// import com.qualcomm.robotcore.robot.Robot;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;
// import java.util.List;
// import org.firstinspires.ftc.robotcore.external.JavaUtil;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.vision.VisionPortal;
// import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
// import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
// import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// import org.firstinspires.ftc.robotcore.external.navigation.*;
// 
// @Autonomous(name = "Auto - Single Step Test")
// public class OneStepAuto extends LinearOpMode {
// private IMU imu;
//  
//   
//   private Servo elbowServo;
//   private Servo clawServo;
//   private Servo extenderServo;
//   private Servo lifterServo;
//   private Servo launcherServo;
//   
//   private DistanceSensor distanceForward;
//   private ColorSensor colorDown;
//   private TouchSensor armMagnetC0;
// private int propLocation = 0;
//   
//   boolean USE_WEBCAM;
//   TfodProcessor myTfodProcessor;
//   
//   AprilTagProcessor myAprilTagProcessor;
//   VisionPortal myVisionPortal;
//   YawPitchRollAngles orientation;
//     AngularVelocity angularVelocity;
//   /**
//    * This function is executed when this OpMode is selected from the Driver Station.
//    */
//   @Override
//   public void runOpMode() {
// 
//     
//     
// 
//     
//     
// /* Servo Config  *********************************************  Servo Config */
//     
//     // Match to config file
//     elbowServo = hardwareMap.get(Servo.class, "elbowServoC0");
//     clawServo = hardwareMap.get(Servo.class, "clawServoC1");
//     extenderServo = hardwareMap.get(Servo.class, "extenderServoC2");
//     lifterServo = hardwareMap.get(Servo.class, "lifterServoC3");
//     launcherServo = hardwareMap.get(Servo.class, "launcherServoC4");
//     imu = hardwareMap.get(IMU.class, "imu");
//     
//     // Set Ranges
//     elbowServo.scaleRange(0, 1);
//     clawServo.scaleRange(0, 1);
//     extenderServo.scaleRange(0, 1);
//     lifterServo.scaleRange(0, 1);
//     launcherServo.scaleRange(0, 1);
//     
//    // Set Directions
//     elbowServo.setDirection(Servo.Direction.REVERSE);
//    
//     ElapsedTime myTimer = new ElapsedTime();
//     ElapsedTime myElapsedTime = new ElapsedTime();
//    elbowServo.setPosition(.15);//retract
//    clawServo.setPosition(.65);
//   // elbowServo.setPosition(.5);
//    sleep(500);
// //  elbowServo.setPosition(0);
//     extenderServo.setPosition(1.0);
//     lifterServo.setPosition(1.0);
//     launcherServo.setPosition(1.0);
//     // Put initialization blocks here.
//     myTimer = new ElapsedTime();
//     
//        
// /* Sensor Config  *****************************************  Sensor Config */    
//     
//     distanceForward = hardwareMap.get(DistanceSensor.class, "distanceForwardC1");
//     colorDown = hardwareMap.get(ColorSensor.class, "colorDownC2");
//     armMagnetC0 = hardwareMap.get(TouchSensor.class, "armMagnetC0");    
// /* Webcam Config androidTextToSpeech.initialize(); *********************************************  Webcam Config */    
//     
//     USE_WEBCAM = true;
//    // initAprilTag();
//     
// /* Init Telemetry  *****************************************  Init Telemetry */
//    initAprilTag();
//     initTfod();
//     waitForStart();
//     imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
//     
//     if (opModeIsActive()) {
//       DriveMotors motors = new DriveMotors(hardwareMap,imu,this); // frontLeftDrive, frontRightDrive,backLeftDrive, backRightDrive);
//       Launcher drone = new Launcher(launcherServo);
//       Claw claw = new Claw(clawServo);
//       RoboCam camera1 = new RoboCam(myAprilTagProcessor,myVisionPortal,motors);
//     //  PurplePixelPlacer ppp = new PurplePixelPlacer();
//       //   Arm arm = new Arm(armMotorLeft, armMotorRight,elbowServo,armMagnetC0);
//      Arm arm = new Arm(elbowServo,armMagnetC0,hardwareMap);
//      try{
//  //   camera1.lineUp(4,this);
//  //   sleep(1000);
//  //     camera1.lineUp(5,this);
//  //     sleep(1000);
//     //  camera1.approach(5,this);
//      int targetTag = 6;
//      boolean isFound = false;
//      double direction = .25;
//      int degreesTurned = 0;
//      while(opModeIsActive()&&!isFound){
//        if(camera1.lineUp(targetTag,this)){
//          isFound=true;
//          motors.allStop();
//        }
//       
//      }
//      
//      sleep(1000);
//      isFound = false;
//      int turn=2;
//      boolean turnRight=true;
//      while(opModeIsActive()&&!isFound){
//       if(camera1.approach(targetTag,this)){
//         camera1.squareUp(targetTag,this);
//         dropPixel(arm,claw);
//       }
//       else{
//         telemetry.addLine("NOT found");
//         telemetry.update();
//       }
//      }
//      }
//      catch(ExecutionException e){}
//      catch(InterruptedException e){}
//      sleep(1000);
//      
//     // catch(){}
//   }
//  
// }
//   private void initAprilTag() {
//     AprilTagProcessor.Builder myAprilTagProcessorBuilder;
//     VisionPortal.Builder myVisionPortalBuilder;
// 
//     // First, create an AprilTagProcessor.Builder.
//     myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
//   //  myAprilTagProcessorBuilder.setLensIntrinsics(835.112, 835.112, 322.08, 242.678);
//     
//     // Create an AprilTagProcessor by calling build.
//     myAprilTagProcessor = myAprilTagProcessorBuilder.build();
//     // Next, create a VisionPortal.Builder and set attributes related to the camera.
//     myVisionPortalBuilder = new VisionPortal.Builder();
//     if (USE_WEBCAM) {
//       // Use a webcam.
//       myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//     } else {
//       // Use the device's back camera.
//       myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
//     }
//     
//     // Add myAprilTagProcessor to the VisionPortal.Builder.
//     myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
//     // Create a VisionPortal by calling build.
//     myVisionPortal = myVisionPortalBuilder.build();
//   }
//   
//   private void initTfod() {
//     TfodProcessor.Builder myTfodProcessorBuilder;
//     VisionPortal.Builder myVisionPortalBuilder;
// 
//     // First, create a TfodProcessor.Builder.
//     myTfodProcessorBuilder = new TfodProcessor.Builder();
//     // Create a TfodProcessor by calling build.
//     myTfodProcessor = myTfodProcessorBuilder.build();
//     // Next, create a VisionPortal.Builder and set attributes related to the camera.
//     myVisionPortalBuilder = new VisionPortal.Builder();
//     if (USE_WEBCAM) {
//       // Use a webcam.
//       myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//     } else {
//       // Use the device's back camera.
//       myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
//     }
//     // Add myTfodProcessor to the VisionPortal.Builder.
//     myVisionPortalBuilder.addProcessor(myTfodProcessor);
//     // Create a VisionPortal by calling build.
//     myVisionPortal = myVisionPortalBuilder.build();
//   }
// 
//   /**
//    * Display info (using telemetry) for a detected object
//    */
//   private int telemetryTfod() {
//     List<Recognition> myTfodRecognitions;
//     Recognition myTfodRecognition;
//     float x;
//     float y;
//     int pixelLocation = 0;
// 
//     // Get a list of recognitions from TFOD.
//     myTfodRecognitions = myTfodProcessor.getRecognitions();
//     telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
//     // Iterate through list and call a function to display info for each recognized object.
//     for (Recognition myTfodRecognition_item : myTfodRecognitions) {
//       myTfodRecognition = myTfodRecognition_item;
//       // Display info about the recognition.
//       telemetry.addLine("");
//       // Display label and confidence.
//       // Display the label and confidence for the recognition.
//       telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
//       // Display position.
//       x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
//       y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
//       if (x < 200) {
//         propLocation = 1;
//       } else if (x > 400) {
//         propLocation = 3;
//       } else {
//         propLocation = 2;
//       }
//       // Display the position of the center of the detection boundary for the recognition
//       telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
//       // Display size
//       // Display the size of detection boundary for the recognition
//       telemetry.addData("- Line: ", JavaUtil.formatNumber(pixelLocation, 0) + "" + "");
//     }
//     return pixelLocation;
//   }
// 
//   public void dropPixel(Arm arm, Claw claw){
//       telemetry.addLine("found");
//       telemetry.update();
//      // isFound=true;
//       arm.moveSteps(300,.8);
//       sleep(1000);
//       arm.setElbowPosition(.6);
//       sleep(1000);
//       claw.openClaw();
//       sleep(1000);
//       arm.setElbowPosition(.1);
//       sleep(1000);
//   }
// }
// 