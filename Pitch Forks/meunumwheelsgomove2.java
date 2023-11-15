package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class mecunumwheelsgomove2 extends LinearOpMode {
 
        private Servo scoopright;
        private Servo scoopleft;
        private Servo drone;
        private Servo pulley;
        private double currentArmPower;
        private double armpwrmultiply;
        
    public void runOpMode() throws InterruptedException {
        
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftfrontdrive");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftreardrive");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontrightdrive");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightreardrive");
        DcMotor leftarm = hardwareMap.dcMotor.get("leftarm");
        DcMotor rightarm = hardwareMap.dcMotor.get("rightarm");
        scoopright = hardwareMap.get(Servo.class, "scoopright");
        scoopleft = hardwareMap.get(Servo.class, "scoopleft");
        drone = hardwareMap.get(Servo.class, "drone");
        pulley = hardwareMap.get(Servo.class, "pulley");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        waitForStart( );

        if (isStopRequested()) return;

        while (opModeIsActive()) {
          telemetry.addData("Arm Position", rightarm.getCurrentPosition());
            double armpos = rightarm.getCurrentPosition();
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower *.6);
            backLeftMotor.setPower(backLeftPower *.6);
            frontRightMotor.setPower(frontRightPower * .6 );
            backRightMotor.setPower(backRightPower*.6 );
            
          if (gamepad2.left_bumper) {
          scoopright.setPosition(0);
          scoopleft.setPosition(1);
        } else if (gamepad2.right_bumper) {
          scoopright.setPosition(1);
          scoopleft.setPosition(0);
        }
        
        if(gamepad2.left_stick_y > .02 || gamepad2.left_stick_y < -.02)
        {
          leftarm.setPower(gamepad2.left_stick_y * .6);
          rightarm.setPower(gamepad2.left_stick_y * .6);
        }else if(armpos < -30 && armpos > -340)
        {
          if(armpos < -270)
          {
            leftarm.setPower(0.2);
            rightarm.setPower(0.2);
          }else if(armpos > -270)
          {
            leftarm.setPower(-0.2);
            rightarm.setPower(-0.2);
          }
          
        }else if(armpos < -340)
        {
          leftarm.setPower(.4);
          rightarm.setPower(.4);
        }
        
        if (gamepad2.triangle && gamepad2.square) {
          drone.setPosition(1);
        }
        
       if (gamepad2.circle) {
          pulley.setPosition(1);
        }
        telemetry.update();
      }
        
    }
}
