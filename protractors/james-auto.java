package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
//THIS BUILD CODE IS NOT FINISHED DO NOT UPLOAD TO ANY BUILDS WITHOUT COMMENTING IT OUT
public class JamesAuto
{
   DcMotor FrontRight;
   DcMotor backRight;
   DcMotor FrontLeft;
   DcMotor backLeft;
   DcMotor lift;
   DcMotor pully;
   Servo clawRight;
   Servo clawLeft;
   Servo launcher;
   DistanceSensor dsensor;

    public void runOpMode() throws InterruptedException {

        /// Forward is Postive
      leftTarget = (int) (FrontLeft.getCurrentPosition() + leftInches * DRIVE_COUNTS_PER_IN);
      rightTarget = (int) (FrontRight.getCurrentPosition() + rightInches * DRIVE_COUNTS_PER_IN);
      backLeftTarget = (int) (BackLeft.getCurrentPosition() + leftInches * DRIVE_COUNTS_PER_IN);
      backRightTarget = BackRight.getCurrentPosition() + rightInches * DRIVE_COUNTS_PER_IN;
      FrontLeft.setTargetPosition(leftTarget);
      FrontRight.setTargetPosition(rightTarget);
      BackLeft.setTargetPosition(backLeftTarget);
      BackRight.setTargetPosition((int) backRightTarget);
      FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontLeft.setPower(power);
      FrontRight.setPower(power);
      BackLeft.setPower(power);
      BackRight.setPower(power);
      while (opModeIsActive() && (FrontLeft.isBusy() || FrontRight.isBusy() || BackLeft.isBusy() || BackRight.isBusy())) {
      }
      FrontLeft.setPower(0);
      FrontRight.setPower(0);
      BackLeft.setPower(0);
      BackRight.setPower(0);
    }
 

  /**
   * Describe this function...
   */
  private void rotate(double power, int degrees) {
    if (opModeIsActive()) {
      // Right is Positive
      leftTarget = (int) (FrontLeft.getCurrentPosition() + (degrees / 6) * DRIVE_COUNTS_PER_IN);
      rightTarget = (int) (FrontRight.getCurrentPosition() - (degrees / 6) * DRIVE_COUNTS_PER_IN);
      backLeftTarget = (int) (BackLeft.getCurrentPosition() + (degrees / 6) * DRIVE_COUNTS_PER_IN);
      backRightTarget = BackRight.getCurrentPosition() - (degrees / 6) * DRIVE_COUNTS_PER_IN;
      FrontLeft.setTargetPosition(leftTarget);
      FrontRight.setTargetPosition(rightTarget);
      BackLeft.setTargetPosition(backLeftTarget);
      BackRight.setTargetPosition((int) backRightTarget);
      FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontLeft.setPower(power);
      FrontRight.setPower(power);
      BackLeft.setPower(power);
      BackRight.setPower(power);
      while (opModeIsActive() && (FrontLeft.isBusy() || FrontRight.isBusy() || BackLeft.isBusy() || BackRight.isBusy())) {
      }
      FrontLeft.setPower(0);
      FrontRight.setPower(0);
      BackLeft.setPower(0);
      BackRight.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void strafe(double power, double rightInches, double leftInches) {
    if (opModeIsActive()) {
      // Right is Positive
      leftTarget = (int) (FrontLeft.getCurrentPosition() + leftInches * DRIVE_COUNTS_PER_IN * 1.1);
      rightTarget = (int) (FrontRight.getCurrentPosition() - rightInches * DRIVE_COUNTS_PER_IN * 1.1);
      backLeftTarget = (int) (BackLeft.getCurrentPosition() - leftInches * DRIVE_COUNTS_PER_IN * 1.1);
      backRightTarget = BackRight.getCurrentPosition() + rightInches * DRIVE_COUNTS_PER_IN * 1.1;
      FrontLeft.setTargetPosition(leftTarget);
      FrontRight.setTargetPosition(rightTarget);
      BackLeft.setTargetPosition(backLeftTarget);
      BackRight.setTargetPosition((int) backRightTarget);
      FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontLeft.setPower(power);
      FrontRight.setPower(power);
      BackLeft.setPower(power);
      BackRight.setPower(power);
      while (opModeIsActive() && (FrontLeft.isBusy() || FrontRight.isBusy() || BackLeft.isBusy() || BackRight.isBusy())) {
      }
      FrontLeft.setPower(0);
      FrontRight.setPower(0);
      BackLeft.setPower(0);
      BackRight.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void liftPully(double power, int inches)
  {
    double armTarget;

    if (opModeIsActive())
    {
      // Right is Positive
    //  armTarget = liftAsDcMotor.getCurrentPosition() - inches * 26.5;
    //  liftAsDcMotor.setTargetPosition((int) armTarget);
    //  liftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //  liftAsDcMotor.setPower(power);
      while (opModeIsActive() && liftAsDcMotor.isBusy())
      {
       
      }
    }
  }

  /**
   * Describe this function...
   */
  private void input()
  {
    rightliftservoAsServo.setPosition(0.5);
  }

  /**
   * Describe this function...
   */
  private void output()
  {
    rightliftservoAsServo.setPosition(0.3);
  }

}
