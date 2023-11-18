package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Arm and Crane")
public class CraneStyle extends LinearOpMode 
{

  private DcMotor FrontRight;
  private DcMotor backRight;
  private DcMotor FrontLeft;
  private DcMotor backLeft;
  private DcMotor lift;
  private DcMotor pully;
  //private Servo clawRight;
 // private Servo clawLeft;
  private Servo launcher;
  private Servo hammer;
  private DistanceSensor dsensor;
  private ColorSensor csensor;
  private TouchSensor touch1;
  private DcMotor pullyMotor;
  private Servo servo1;
  

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
    pullyMotor = hardwareMap.get(DcMotor.class, "pullyMotor");
    //clawRight = hardwareMap.get(Servo.class, "clawRight");
    //clawLeft = hardwareMap.get(Servo.class, "clawLeft");
    launcher = hardwareMap.get(Servo.class, "launcherServo");
    servo1 = hardwareMap.get(Servo.class, "hammerServo");
    dsensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
    csensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    touch1 = hardwareMap.get(TouchSensor.class, "touch1");
  // Put initialization blocks here.
  backRight.setDirection(DcMotorSimple.Direction.REVERSE);
  backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
  
    waitForStart();
    if (opModeIsActive()) {  
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.update();
        
        
        if (gamepad2.left_bumper) {
          telemetry.addData("Finger 1", "x=" + JavaUtil.formatNumber(gamepad1.touchpad_finger_1_x, 2) + " y=" + JavaUtil.formatNumber(gamepad1.touchpad_finger_1_y, 2));
          servo1.setPosition(0.7);
        } else {
          telemetry.addData("Finger 1", "");
        }
        // Display finger 2 x & y position if finger detected
        if (gamepad2.right_bumper) {
          servo1.setPosition(1);
          telemetry.addData("Finger 2", "x=" + JavaUtil.formatNumber(gamepad1.touchpad_finger_2_x, 2) + " y=" + JavaUtil.formatNumber(gamepad1.touchpad_finger_2_y, 2));
        } else {
          telemetry.addData("Finger 2", "");
        }
        
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
        if (gamepad2.dpad_left){
          launcher.setPosition(1);
        }
        if (gamepad2.dpad_right){
          launcher.setPosition(0);
        }
        if (touch1.isPressed()&&!gamepad2.right_stick_button) 
        {
          pullyMotor.setPower(0);
        }
        else 
        {
          pullyMotor.setPower(gamepad2.right_stick_y);
        }
        double value = dsensor.getDistance(DistanceUnit.INCH);
        //Lift Controls
        lift.setPower(gamepad2.left_stick_y);
        //pully.setPower(gamepad2.right_stick_y);
        telemetry.addData("FLeft Pow", FrontLeft.getPower());
        telemetry.addData("BLeft Pow", FrontLeft.getPower());
        telemetry.addData("FRight Pow", FrontLeft.getPower());
        telemetry.addData("BRight Pow", FrontLeft.getPower());
        //telemetry.addData("clawL Pos", clawLeft.getPosition());
        //telemetry.addData("clawR Pos", clawRight.getPosition());
        telemetry.addData("Launcher Pos", launcher.getPosition());
        telemetry.addData("Distance: ", value);
        telemetry.update();
      }
    }
  }
}
     
