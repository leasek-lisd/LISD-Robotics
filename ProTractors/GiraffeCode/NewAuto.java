// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
// @Autonomous(name = "New Auto")
// public class NewAuto extends LinearOpMode 
// {
// private IMU imu;
// 
//   //This function is executed when this OpMode is selected from the Driver Station.
//   @Override
//   public void runOpMode() 
//   {
//   imu = hardwareMap.get(IMU.class, "imu");
//     
//   /* Servo Config  *********************************************  Servo Config */
//   DriveMotors motors = new DriveMotors(hardwareMap,imu, this);  // frontLeftDrive, frontRightDrive,backLeftDrive, backRightDrive);
//   PurplePixelPlacer ppp = new PurplePixelPlacer(hardwareMap);
//    
//   /* Sensor Config  *****************************************  Sensor Config */    
//     
//   /* Webcam Config androidTextToSpeech.initialize(); *********************************************  Webcam Config */    
//   
// waitForStart();
//   imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
//     
//     if (opModeIsActive()) 
//     {
//       motors.leftInches(9);
//       motors.forwardInches(-15);
//       motors.allStop();
//     }
//   }
// }
// 