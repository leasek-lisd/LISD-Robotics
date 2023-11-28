package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class DriveMotors extends DriveTrain {
  
  public DriveMotors(HardwareMap hardwareMap)
    /*    DcMotor inFrontLeftDrive,
        DcMotor inFrontRightDrive,
        DcMotor inBackLeftDrive,
        DcMotor inBackRightDrive
    )*/{
      super(hardwareMap);  
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
  
