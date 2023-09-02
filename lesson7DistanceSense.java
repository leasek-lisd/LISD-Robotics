
/***********************************************************************
*                                                                      *
* OnbotJava Editor is still : beta! Please inform us of any bugs       |
* on our discord channel! https://discord.gg/e7nVjMM                   *
* Only BLOCKS code is submitted when in Arena                          *
*                                                                      *
***********************************************************************/


public class MyFIRSTJavaOpMode extends LinearOpMode {
    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor frontLeft;
    DcMotor frontRight;
    ColorSensor color1;
    DistanceSensor distance1;
    BNO055IMU imu;

@Override
    public void runOpMode() {
      motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
      motorRight = hardwareMap.get(DcMotor.class, "motorRight");
      frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
      frontRight = hardwareMap.get(DcMotor.class, "frontRight");
      color1 = hardwareMap.get(ColorSensor.class, "color1");
      distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
      imu = hardwareMap.get(BNO055IMU.class, "imu");
      // Put initialization blocks here
      waitForStart();
      // Put run blocks here
      sleep(1000);
      motorLeft.setDirection(DcMotor.Direction.REVERSE);
      motorLeft.setPower(0.5);
      motorRight.setPower(0.5);
      while (opModeIsActive()) {
        // Put loop blocks here
        telemetry.addData("Distance", (distance1.getDistance(DistanceUnit.CM)));
        telemetry.update();
        if (40 >= (distance1.getDistance(DistanceUnit.CM))) {
          break;
        }
      }
      motorLeft.setPower(0);
      motorRight.setPower(0);
      sleep(200);
      rotateLeft(200);
      forward(400);
    }
    public void rotateLeft(int turnTime){
        motorLeft.setPower(-.6);
        motorRight.setPower(.6);
        sleep(turnTime);
        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(100);
      }
      public void forward(int runTime){
        motorLeft.setPower(1);
        motorRight.setPower(1);
        sleep(runTime);
        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(100);
      }
}
