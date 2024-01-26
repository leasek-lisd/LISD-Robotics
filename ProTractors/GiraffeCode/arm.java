package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
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

public class arm 
{
  DcMotor leftLiftMotor;
  DcMotor rightLiftMotor;
  DcMotor spinner;
  DcMotor pully;
  Servo leftPin;
  Servo rightPin;
  Servo bucket;
  
  public arm(HardwareMap hardwareMap)
  {
    leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftE3");
    rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftC3");
    spinner = hardwareMap.get(DcMotor.class, "spinnerC0");
    pully = hardwareMap.get(DcMotor.class, "pullyE0");
    bucket = hardwareMap.get(Servo.class, "bucket");
    leftPin = hardwareMap.get(Servo.class, "leftPin");
    rightPin = hardwareMap.get(Servo.class, "rightPin");
  }
//Operation Methods
  public void spinToWin(double direction)
  {
    spinner.setPower(direction);
  }
  public void lineOpsBackward(double power) 
  {
    pully.setPower(power);
  }
  public void lineOpsForward(double power)
  {
    pully.setPower(-power);
  }
  public void placeUp()
  {
    bucketUp();
    liftAutoOps(500);
  }
  public void foldBack()
  {
    liftAutoOps(-200);
    bucketDown();
  }
  public void liftManualOps(double input)
  {
    leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftLiftMotor.setPower(input);
    rightLiftMotor.setPower(input);
  }
  public void liftAutoOps(int targetTick)
  {
    rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightLiftMotor.setTargetPosition(targetTick);
    leftLiftMotor.setTargetPosition(targetTick);
    rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightLiftMotor.setPower(0.7);
    leftLiftMotor.setPower(0.7);
  }
  public void holding(int input)
  {
    rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightLiftMotor.setTargetPosition(input);
    leftLiftMotor.setTargetPosition(input);
    rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightLiftMotor.setPower(1);
    leftLiftMotor.setPower(1);
    while(rightLiftMotor.isBusy() || leftLiftMotor.isBusy())
    {//DO NOT PUT ANYTHING IN HERE
    }
    rightLiftMotor.setPower(0);
    leftLiftMotor.setPower(0);
  }
//Supporting Methods
  public void port(double check)
  {
    leftPin.setPosition(check);
  }
  public void starboard(double check)
  {
    rightPin.setPosition(check);
  }
  public void bucketUp()
  {
    bucket.setPosition(1);
  }
  public void bucketDown()
  {
    bucket.setPosition(0);
  }
//Telemetry Methods
  public double getRightPin()
  {
    return rightPin.getPosition();
  }
  public double getLeftPin()
  {
    return leftPin.getPosition();
  }
}
  
