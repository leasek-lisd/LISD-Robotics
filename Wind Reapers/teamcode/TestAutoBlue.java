// /*
// Copyright 2023 FIRST Tech Challenge Team FTC
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial
// portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// */
// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import org.firstinspires.ftc.vision.VisionPortal;
// import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
// import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// 
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.qualcomm.robotcore.hardware.TouchSensor;
// import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// import org.firstinspires.ftc.robotcore.external.navigation.*;//YawPitchRollAngles;
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.qualcomm.robotcore.hardware.TouchSensor;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import java.util.concurrent.TimeUnit;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
// import java.util.List;
// 
// @Autonomous
// 
//   public class TestAutoBlue extends LinearOpMode {
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
//   private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
//     private AprilTagDetection desiredTag = null; 
//     private VisionPortal visionPortal; 
//   
//   boolean USE_WEBCAM;
// AprilTagProcessor myAprilTagProcessor;
//   VisionPortal myVisionPortal;
//     @Override
//     public void runOpMode() {
//       elbowServo = hardwareMap.get(Servo.class, "elbowServoC0");
//       clawServo = hardwareMap.get(Servo.class, "clawServoC1");
//       extenderServo = hardwareMap.get(Servo.class, "extenderServoC2");
//       lifterServo = hardwareMap.get(Servo.class, "lifterServoC3");
//       launcherServo = hardwareMap.get(Servo.class, "launcherServoC4");
//       imu = hardwareMap.get(IMU.class, "imu");
//       
//       
//       elbowServo.scaleRange(0, 1);
//       clawServo.scaleRange(0, 1);
//       extenderServo.scaleRange(0, 1);
//       lifterServo.scaleRange(0, 1);
//       launcherServo.scaleRange(0, 1);
//       
//       elbowServo.setDirection(Servo.Direction.REVERSE);
//       
//       
//       int tagNumber = 0;  
//       distanceForward = hardwareMap.get(DistanceSensor.class, "distanceForwardC1");
//       colorDown = hardwareMap.get(ColorSensor.class, "colorDownC2");
//       armMagnetC0 = hardwareMap.get(TouchSensor.class, "armMagnetC0");
//       initAprilTag();
//         telemetry.addData("Status", "Initialized");
//         telemetry.update();
//         // Wait for the game to start (driver presses PLAY)
//         waitForStart();
//           imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
//           DriveMotors motors = new DriveMotors(hardwareMap,imu, this); // frontLeftDrive, frontRightDrive,backLeftDrive, backRightDrive);
//           Launcher drone = new Launcher(launcherServo);
//           Claw claw = new Claw(clawServo);
//           Arm arm = new Arm(elbowServo,armMagnetC0,hardwareMap);
//           
//           // need to put code below into contructors
//           elbowServo.setPosition(.15);//retract
//           clawServo.setPosition(.65);
//           extenderServo.setPosition(1.0);
//           lifterServo.setPosition(1);// up is zero
//           launcherServo.setPosition(1.0);
//           RoboCam camera1 = new RoboCam(myAprilTagProcessor,myVisionPortal,motors);
//          // motors.forwardInches(12);
//           telemetry.addData("Motors Busy? ", motors.areBusy());
//             telemetry.update();
//        // motors.forwardInches(6);
//       // motors.rightInches(4);
//       // motors.backwardInches(45);
//       //  sleep(5000);
//         boolean skip = false;
//         
//       
//         imu.resetYaw();
//        
//        
//     boolean found = false;
//     int blueValue = 300;
//     int redValue = 300;
// 
//       if(skip||distanceForward.getDistance(DistanceUnit.INCH)<28&&distanceForward.getDistance(DistanceUnit.INCH)>21){
//         telemetry.addLine("Middle Line Found");
//         telemetry.update();
//         // motors.leftInches(4);
//         sleep(250);
//         motors.forwardInches(41);
//         //sleep(1000);
//         motors.forwardInches(15,false);
//         //sleep(2000);
//         found = false;
//         while(motors.areBusy()&&opModeIsActive()){
//             //sleep(200);
//             if(colorDown.red() > redValue){
//              motors.allStop();
//              motors.backwardInches(1.5);
//              lifterServo.setPosition(0);
//              sleep(30000);
//              telemetry.addLine("found color");
//             telemetry.update();
//             motors.backwardInches(1);
//             found = true;
//             tagNumber = 5;
//             }
//             else{
//                 telemetry.addLine("looking for color " + colorDown.red());
//                 telemetry.update();
//             }
//         }
//         if(!found){
//           motors.backwardInches(8,false);
//         while(motors.areBusy()&&opModeIsActive()){
//         if(colorDown.red() > redValue){ //750
//              motors.allStop();
//              lifterServo.setPosition(0);
//              sleep(30000);
//              telemetry.addLine("found color");
//             telemetry.update();
//             motors.forwardInches(2);
//             found = true;
//             tagNumber = 5;
//             }
//             else{
//                 telemetry.addLine("looking for color " + colorDown.red());
//             telemetry.update();
//             }
//           
//         }
//         }
//         telemetry.addLine("done moving" + found);
//             telemetry.update();
//         sleep(5000);
//       } 
//       else{
//         telemetry.addLine("Middle Line Not Found");
//         motors.forwardInches(2);
//         motors.rightInches(24,false);
//         
//         while(motors.areBusy()&&opModeIsActive()){
//           if(distanceForward.getDistance(DistanceUnit.INCH)<22&&distanceForward.getDistance(DistanceUnit.INCH)>18){
//             motors.allStop();
//             telemetry.addLine("Left Line Found");
//             telemetry.update();
//             motors.leftInches(13);
//            telemetry.addLine("here");
//             telemetry.update();
//             motors.forwardInches(32);
//              while(motors.areBusy()&&opModeIsActive()){
//         if(colorDown.blue() > redValue){
//           motors.allStop();
//              telemetry.addLine("found color");
//             telemetry.update();
//             sleep(200);
//             found = true;
//             tagNumber=4;
//             lifterServo.setPosition(0);
//             sleep(30000);
//              motors.rightInches(5);
//                  sleep(100);
//                  motors.forwardInches(30);
//                  motors.rotateRightDegrees(90,.25);
//                  motors.forwardInches(60);
//         }
//              }
//              
//           // Look for the line on each side   
//           if(!found){ 
//             motors.leftInches(6,false);
//              while(motors.areBusy()&&opModeIsActive()){
//                if(colorDown.blue() > redValue){
//                  motors.allStop();
//                  lifterServo.setPosition(1);
//                  telemetry.addLine("found color");
//                  telemetry.update();
//                  sleep(200);
//                  found = true;
//                  tagNumber=4;
//                  lifterServo.setPosition(0);
//                  sleep(30000);
//                  motors.rightInches(5);
//                  sleep(100);
//                 // motors.forwardInches(30);
//                 // motors.rotateRightDegrees(90,.25);
//                 // motors.forwardInches(60);
//                 }
//               }
//             
//           }
//           
//           // blue needs to be 1500
//             telemetry.update();
//           //  sleep(5000);
//           }
//         }
//         
//         
//         
//       } 
//       
//       if(!found&&opModeIsActive()){  // right line
//       telemetry.addLine("looking for right line");
//       telemetry.update();
//         motors.forwardInches(27);
//        // sleep(500);
//         tagNumber = 6;
//         motors.leftInches(29);
//          //while(motors.areBusy()){
//         //       if(colorDown.red() > redValue){
//         //         motors.allStop();
//                  
//         //       }
//       sleep(100);
//                lifterServo.setPosition(0);
//                sleep(30000);
//                  found=true;
//          //}
//       }
//   //    if(tagNumber = 4){
//         telemetry.addLine("Moving to 4");
//         telemetry.update();
//         motors.forwardInches(24);
//           motors.rotateRightDegrees(90,.75);
//           motors.forwardInches(36);
//           motors.rightInches(24);
//     //  }
//       //if(found){
//    //     moveToBoard(motors,tagNumber, false);
//       //}
//     }
//     
//     public void moveToBoard(DriveMotors motors, int tagNumber, boolean close){
//       double turnSpeed = .75;
//       switch(tagNumber){
//         case 0:
//           break;
//         case 1:
//         case 2:
//         case 3:
//           motors.rotateRightDegrees(179,.75);
//           //dropPixel(tagNumber);
//           break;
//         case 4:
//           motors.forwardInches(24);
//           motors.rotateRightDegrees(90,turnSpeed);
//           motors.forwardInches(36);
//           motors.rightInches(24);
//           break;
//         case 5:
//           motors.forwardInches(12);
//           motors.rotateRightDegrees(90,turnSpeed);
//           motors.forwardInches(30);
//           motors.rightInches(24);
//           break;
//         case 6:
//           motors.leftInches(12);
//           motors.forwardInches(24);
//           motors.rotateRightDegrees(90,turnSpeed);
//           motors.forwardInches(28);
//           motors.rightInches(24);
//           break;
//       
//       }
//     }
//     
//     
//     
//     
//     
//       public double getYaw(){
//       YawPitchRollAngles orientation;
//       
//       orientation = imu.getRobotYawPitchRollAngles();
//       return orientation.getYaw(AngleUnit.DEGREES);
//     }
//      
//     
//   private void initAprilTag() {
//         // Create the AprilTag processor by using a builder.
//         aprilTag = new AprilTagProcessor.Builder().build();
// 
//         // Adjust Image Decimation to trade-off detection-range for detection-rate.
//         // eg: Some typical detection data using a Logitech C920 WebCam
//         // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//         // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//         // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
//         // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
//         // Note: Decimation can be changed on-the-fly to adapt during a match.
//         aprilTag.setDecimation(2);
// 
//         // Create the vision portal by using a builder.
//         if (USE_WEBCAM) {
//             visionPortal = new VisionPortal.Builder()
//                     .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                     .addProcessor(aprilTag)
//                     .build();
//         } else {
//             visionPortal = new VisionPortal.Builder()
//                     .setCamera(BuiltinCameraDirection.BACK)
//                     .addProcessor(aprilTag)
//                     .build();
//         }
//     }
// 
//     /*
//      Manually set the camera gain and exposure.
//      This can only be called AFTER calling initAprilTag(), and only works for Webcams;
//     */
//     private void    setManualExposure(int exposureMS, int gain) {
//         // Wait for the camera to be open, then use the controls
// 
//         if (visionPortal == null) {
//             return;
//         }
// 
//         // Make sure camera is streaming before we try to set the exposure controls
//         if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//             telemetry.addData("Camera", "Waiting");
//             telemetry.update();
//             while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                 sleep(20);
//             }
//             telemetry.addData("Camera", "Ready");
//             telemetry.update();
//         }
// 
//         // Set camera controls unless we are stopping.
//         if (!isStopRequested())
//         {
//             ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//             if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                 exposureControl.setMode(ExposureControl.Mode.Manual);
//                 sleep(50);
//             }
//             exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//             sleep(20);
//             GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//             gainControl.setGain(gain);
//             sleep(20);
//         }
//     }
//     
//     
// }
// 