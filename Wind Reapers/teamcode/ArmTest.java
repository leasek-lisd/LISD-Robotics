// /*
// Copyright 2023 FIRST Tech Challenge Team 24630
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
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// import org.firstinspires.ftc.robotcore.external.navigation.*;//AngularVelocity;
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
// /**
//  * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
//  * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
//  * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
//  * class is instantiated on the Robot Controller and executed.
//  *
//  * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
//  * It includes all the skeletal structure that all linear OpModes contain.
//  *
//  * Remove a @Disabled the on the next line or two (if present) to add this OpMode to the Driver Station OpMode list,
//  * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
//  */
// @TeleOp
// 
// public class ArmTest extends LinearOpMode {
//   private IMU imu;
// 
//     @Override
//     public void runOpMode() {
// imu = hardwareMap.get(IMU.class, "imu");
//         telemetry.addData("Status", "Initialized");
//         telemetry.update();
//         // Wait for the game to start (driver presses PLAY)
//         waitForStart();
//         GreatArm arm = new GreatArm(hardwareMap);
//         DriveMotors motors = new DriveMotors(hardwareMap,imu,this); // frontLeftDrive, frontRightDrive,backLeftDrive, backRightDrive);
//       imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
//     
//         // run until the end of the match (driver presses STOP)
//         while (opModeIsActive()) {
//             telemetry.addData("Status", "Running");
//             telemetry.update();
//             if (gamepad2.left_bumper){
//               arm.moveToPosition(100,.5);
//             }
//             if (gamepad2.right_bumper){
//               arm.moveToPosition(-100,.5);
//             }
//             
//             if (gamepad2.dpad_up){
//               motors.forwardInches(10);
//             }
// 
//         }
//     }
// }
// 