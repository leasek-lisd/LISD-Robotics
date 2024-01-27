package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class GreatArm {

    // todo: write your code here
    private DcMotor upperArm;
    private DcMotor lowerArm;
    
    public GreatArm (HardwareMap hardwareMap){
      lowerArm= hardwareMap.get(DcMotor.class, "upperArmMotorC1");
      upperArm = hardwareMap.get(DcMotor.class, "lowerArmMotorC0");
      upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      lowerArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    public void setUpperArmPower(double power){
       upperArm.setPower(0.3 * power);
      //  lowerArmMotorC0.setPower(0.3 * power);
    }
    public void setLowerArmPower(double power){
      // upperArmMotorC1.setPower(0.3 * power);
        lowerArm.setPower(0.3 * power);
    }
    
    public void moveToPosition(int positionSteps, double speed){
    // 38 steps per rev
    // 80:1 gear ratio


      int upperSteps = positionSteps;
      int lowerSteps = positionSteps;
      
      upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      lowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      
      upperArm.setTargetPosition(upperSteps);
      lowerArm.setTargetPosition(lowerSteps);
      
      upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      lowerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      
      upperArm.setPower(speed);
      lowerArm.setPower(speed);
    }
    
}