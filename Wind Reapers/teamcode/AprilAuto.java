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
// @Autonomous(name = "April Auto")
// public class AprilAuto extends LinearOpMode {
//   private IMU imu;
//   private Servo elbowServo;
//   private Servo clawServo;
//   private Servo extenderServo;
//   private Servo lifterServo;
//   private Servo launcherServo;
//   private DistanceSensor distanceForward;
//   private ColorSensor colorDown;
//   private TouchSensor armMagnetC0;
//   private int propLocation = 0;
//   boolean USE_WEBCAM;
//   AprilTagProcessor myAprilTagProcessor;
//   VisionPortal myVisionPortal;
//   YawPitchRollAngles orientation;
//   AngularVelocity angularVelocity;
//   
//   
//   @Override
//   public void runOpMode() {
//     
//     
//     imu = hardwareMap.get(IMU.class, "imu");
//     distanceForward = hardwareMap.get(DistanceSensor.class, "distanceForwardC1");
//     colorDown = hardwareMap.get(ColorSensor.class, "colorDownC2");
//     USE_WEBCAM = true;
//     initAprilTag();
//     imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
//   
//   waitForStart();   
//   
//   if (opModeIsActive()) {
//     DriveMotors motors = new DriveMotors(hardwareMap,imu,this); // frontLeftDrive, frontRightDrive,backLeftDrive, backRightDrive);
//     
//       RoboCam camera1 = new RoboCam(myAprilTagProcessor,myVisionPortal,motors,this);
//       motors.leftInches(12);
//     //  motors.backwardInches(48);
//       
//      //motors.forwardInches2(2,.25);
//       //motors.leftInches2(16,.25);
//       //motors.rightDegrees(45,.3);
//       //sleep(2000);
//       telemetry.addData("maxPower: ",motors.forwardInches2(-4,.5));
//       telemetry.update();
//       sleep(10000);
//       //motors.backwardInches2(6,.5);
//       /*
//       sleep(5000);
//       
//       
//       double targetX = camera1.getTagX(2);
//       double targetYaw = camera1.getTagYaw(2);
//       double moveDistance = targetX*Math.sin(camera1.toRadians(90-targetYaw))/Math.sin(camera1.toRadians(90));
// 
//       //motors.rightInches(2);
//      // motors.rotateRightDegrees(38,.8);
//       telemetry.addData("Calculated: ", moveDistance);
//       telemetry.addData("x",targetX);
//       telemetry.addData("Yaw",(int)targetYaw);
//       telemetry.update();
//       
//       motors.rotateRightDegrees(-(int)targetYaw,.6);
//       motors.leftInches(moveDistance);
//       sleep(1000);
//       camera1.lineUp(2,this);
//       */
//       sleep(5000);
//      // camera1.lineUp3(2);
//     //  try{
//       //  camera1.lineUp3(2,20);
//     //  }
//    //   catch(InterruptedException e){}
//   //    catch(ExecutionException e){};
//         
//       
//       sleep(15000);
//     }
// }
// 
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
// }
// 