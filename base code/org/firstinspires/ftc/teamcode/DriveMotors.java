package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;


public class DriveMotors extends DriveTrain {
  
  public DriveMotors(
        DcMotor inFrontLeftDrive,
        DcMotor inFrontRightDrive,
        DcMotor inBackLeftDrive,
        DcMotor inBackRightDrive
    ){
      super(inFrontLeftDrive,inFrontRightDrive,inBackLeftDrive,inBackRightDrive);  
   //   frontLeftDrive = inFrontLeftDrive; 
  //    frontRightDrive = inFrontRightDrive;
  //    backLeftDrive = inBackLeftDrive;
  //    backRightDrive = inBackRightDrive;
    
    }
 
  public void frontLeftPower(double power){
    frontLeftDrive.setPower(power);
  }

  public void frontRightPower(double power){
    frontRightDrive.setPower(power);
  }

  public void backLeftPower(double power){
    backLeftDrive.setPower(power);
  }

  public void backRightPower(double power){
    backRightDrive.setPower(power);
  }
}
  
