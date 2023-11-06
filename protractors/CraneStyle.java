package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Arm and Crane")
public class CraneStyle extends LinearOpMode {

  private DcMotor FrontRight;
  private DcMotor backRight;
  private DcMotor FrontLeft;
  private DcMotor backLeft;
  private DcMotor lift;
  private DcMotor pully;
  private Servo clawRight;
  private Servo clawLeft;
  private Servo launcher;
  private DistanceSensor dsensor;
 

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
    backRight = hardwareMap.get(DcMotor.class, "backRight");
    FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    lift = hardwareMap.get(DcMotor.class, "liftMotor");
    pully = hardwareMap.get(DcMotor.class, "pullyMotor");
    clawRight = hardwareMap.get(Servo.class, "clawRight");
    clawLeft = hardwareMap.get(Servo.class, "clawLeft");
    launcher = hardwareMap.get(Servo.class, "launcherServo");
    dsensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
   
  // Put initialization blocks here.
  //backRight.setDirection(DcMotorSimple.Direction.REVERSE);
  //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
 
    waitForStart();
    if (opModeIsActive()) {  
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.update();
       
        FrontLeft.setPower((1 * gamepad1.left_stick_y) + 1 * gamepad1.left_stick_x + 0.7 * gamepad1.right_stick_x);
        FrontRight.setPower(((1 * gamepad1.left_stick_y) - 1 * gamepad1.left_stick_x) + -0.7 * gamepad1.right_stick_x);
        backLeft.setPower(((1 * gamepad1.left_stick_y) - 1 * gamepad1.left_stick_x) + 0.7 * gamepad1.right_stick_x);
        backRight.setPower((1 * gamepad1.left_stick_y) + 1 * gamepad1.left_stick_x + -0.7 * gamepad1.right_stick_x);
       
        if (gamepad1.dpad_right) {
          FrontLeft.setPower(0.25);
          FrontRight.setPower(-0.25);
          backLeft.setPower(0.25);
          backRight.setPower(-0.25);
        }
        if (gamepad1.dpad_left) {
          FrontLeft.setPower(-0.25);
          FrontRight.setPower(0.25);
          backLeft.setPower(-0.25);
          backRight.setPower(0.25);
        }
        if (gamepad1.dpad_up) {
          FrontLeft.setPower(0.25);
          FrontRight.setPower(0.25);
          backLeft.setPower(0.25);
          backRight.setPower(0.25);
        }
        if (gamepad1.dpad_down) {
          FrontLeft.setPower(-0.25);
          FrontRight.setPower(-0.25);
          backLeft.setPower(-0.25);
          backRight.setPower(-0.25);
        }
        //Bucket Controls
        if (gamepad2.right_bumper) {
          clawRight.setPosition(0.58);
          clawLeft.setPosition(0);
        }
        if (gamepad2.left_bumper) {
          clawRight.setPosition(0);
          clawLeft.setPosition(0.48);
        }
        if (gamepad2.y) {
          clawRight.setPosition(0);
        }
        if (gamepad2.a) {
          clawRight.setPosition(0.48);
        }
        if (gamepad2.b) {
          clawLeft.setPosition(0.48);
        }
        if (gamepad2.x) {
          clawLeft.setPosition(0);
        }
        if (gamepad2.dpad_left){
          launcher.setPosition(1);
        }
        if (gamepad2.dpad_right){
          launcher.setPosition(0.2);
        }
        double value = dsensor.getDistance(DistanceUnit.INCH);
        //Lift Controls
        lift.setPower(gamepad2.left_stick_y);
        pully.setPower(gamepad2.right_stick_y);
        telemetry.addData("FLeft Pow", FrontLeft.getPower());
        telemetry.addData("BLeft Pow", FrontLeft.getPower());
        telemetry.addData("FRight Pow", FrontLeft.getPower());
        telemetry.addData("BRight Pow", FrontLeft.getPower());
        telemetry.addData("clawL Pos", clawLeft.getPosition());
        telemetry.addData("clawR Pos", clawRight.getPosition());
        telemetry.addData("Launcher Pos", launcher.getPosition());
        telemetry.addData("Distance: ", value);
        telemetry.update();
      }
    }
  }
}

