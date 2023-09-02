
/***********************************************************************
*                                                                      *
* OnbotJava Editor is still : beta! Please inform us of any bugs       |
* on our discord channel! https://discord.gg/e7nVjMM                   *
* Only BLOCKS code is submitted when in Arena                          *
*                                                                      *
***********************************************************************/
import java.lang.Math;

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
      motorLeft.setDirection(DcMotor.Direction.REVERSE);
      
      sleep(1000);
      telemetry.addData("Distance", (distance1.getDistance(DistanceUnit.CM)));
    //    telemetry.update();
    int i = 1;
    do{
    forward(100);
    i++;
    sleep(500);
    }while(i<=3);
    
    
    do{
      double right = ping();
      rotateRight(400);
     // telemetry.update();
     double left = ping();
     if(left>right && Math.abs(left-right)<10 ){
         forward(50);
     } else
     {
         rotateLeft(400);
         forward(100);
     }
    }while(Math.abs(left-right)>10);
    }
    public void forward(int runTime){
        motorLeft.setPower(1);
        motorRight.setPower(1);
        sleep(runTime);
        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(100);
      }
      
      public void backward(int runTime){
        motorLeft.setPower(1);
        motorRight.setPower(1);
        sleep(runTime);
        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(100);
      }
      
      public void rotateRight(int turnTime){
        motorLeft.setPower(.6);
        motorRight.setPower(-.6);
        sleep(turnTime);
        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(100);
      }
      
      public void rotateLeft(int turnTime){
        motorLeft.setPower(-.6);
        motorRight.setPower(.6);
        sleep(turnTime);
        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(100);
      }
      
      public void veerRight(int turnTime, double offset){
              motorLeft.setPower(1);
              motorRight.setPower(1-offset);
              sleep(turnTime);
              motorLeft.setPower(0);
              motorRight.setPower(0);
              sleep(100);
      }
      
      public void goUntil(int cm){
                  motorLeft.setPower(0.5);
                  motorRight.setPower(0.5);
                  while (opModeIsActive()) {
              // Put loop blocks here
              // telemetry.addData("Distance", (distance1.getDistance(DistanceUnit.CM)));
                  telemetry.update();
                  if (cm >= (distance1.getDistance(DistanceUnit.CM))) {
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                  break;
                  }
                  }
                  
        }
      
      public double ping(){
        telemetry.update();
        return distance1.getDistance(DistanceUnit.CM);
        }
        
      public boolean isRed(){
              telemetry.update();
              return (color1.red() >= 200);
          }
          
          public boolean isBlue(){
              telemetry.update();
              return (color1.blue() >= 200);
    }
}
