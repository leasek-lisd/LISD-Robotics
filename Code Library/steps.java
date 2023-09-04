
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
    int leftPos = 0;
    int rightPos = 0;
    int stepPos = 2100;

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
      // Put run blocks here
      motorLeft.setDirection(DcMotor.Direction.REVERSE);
      motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      
      
      int steps = 3;
      
      forwardSteps(1);
      sleep(1000);
      forwardSteps(2);
      /*motorLeft.setTargetPosition(500);
      motorRight.setTargetPosition(3500);
      waitForStart();
      motorLeft.setPower(-1);
      motorRight.setPower(1);
      */
      
      frontLeft.setPower(1);
      frontRight.setPower(0);
      
      rotateRight();
      forwardSteps(1);
      while (false) {
        // Put loop blocks here
      }
    }
    
    public void rotateRight(){
        leftPos += 1400;
        rightPos -= 1400;
        motorLeft.setTargetPosition(leftPos);
        motorRight.setTargetPosition(rightPos);
        waitForStart();
        motorLeft.setPower(.5);
        motorRight.setPower(.5);
        sleep(1000);
    }
    
    public void forwardSteps(int steps){
        int targetPos = leftPos + steps * stepPos;
        leftPos = targetPos;
        rightPos = targetPos;
        motorLeft.setTargetPosition(targetPos);
        motorRight.setTargetPosition(targetPos);
        waitForStart();
        motorLeft.setPower(1);
        motorRight.setPower(1);
        sleep(1000);
      }
}
