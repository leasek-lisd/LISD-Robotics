package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;


public class Move {
    
    
    
    
    // todo: write your code here
 public static void slideFront(DcMotor leftMotor, DcMotor rightMotor, double x){
    leftMotor.setPower(x*.25);
    rightMotor.setPower(-x*.25);
  }
  
  public static void slideRear(DcMotor leftMotor, DcMotor rightMotor, double x){
    leftMotor.setPower(x*.25);
    rightMotor.setPower(-x*.25);
  }
 public static void forward(DcMotor motors, double speed){
//    motors.frontLeftDrive.setPower(speed);
//    motors.frontRightDrive.setPower(speed);
//    motors.backLeftDrive.setPower(speed);
//    motors.backRightDrive.setPower(speed);
  }
    
    
}