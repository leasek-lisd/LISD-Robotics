
package org.firstinspires.ftc.teamcode;
import java.util.concurrent.ExecutionException;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.*;//YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import java.util.List;

@Autonomous

  public class AutoPID extends LinearOpMode {
  private IMU imu;
  private Servo elbowServo;
  private Servo clawServo;
  private Servo elevationServo;
  private Servo lifterServo;
  private Servo launcherServo;
  private DistanceSensor distanceForward;
  private ColorSensor colorDown;
  private TouchSensor armMagnetC0;
  private int propLocation = 0;
 // private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
 // private AprilTagDetection desiredTag = null; 
 // private VisionPortal visionPortal; 
  private AnalogInput pot1;
  private double turnSpeed = .5;  
  //boolean USE_WEBCAM;
 // AprilTagProcessor myAprilTagProcessor;
 // VisionPortal myVisionPortal;
   
  @Override
  public void runOpMode() {
  // elbowServo = hardwareMap.get(Servo.class, "elbowServoC0");
  //  clawServo = hardwareMap.get(Servo.class, "clawServoC1");
  //  lifterServo = hardwareMap.get(Servo.class, "lifterServoC3");
    imu = hardwareMap.get(IMU.class, "imu");
  //  pot1 = hardwareMap.get(AnalogInput.class, "pot1");
  //  elevationServo = hardwareMap.get(Servo.class, "elevationServoE2");
   // launcherServo = hardwareMap.get(Servo.class, "launcherServoE0");
    DriveMotors motors = new DriveMotors(hardwareMap,imu, this); // frontLeftDrive, frontRightDrive,backLeftDrive, backRightDrive);
   
      
    int tagNumber = 0;  
   
    //clawServo.scaleRange(0, 1);
    //lifterServo.scaleRange(0, 1);
    //launcherServo.scaleRange(0, 1);
    
    distanceForward = hardwareMap.get(DistanceSensor.class, "distanceForwardC1");
    //colorDown = hardwareMap.get(ColorSensor.class, "colorDownC2");
    
    
    //telemetry.addData("Status", "Initialized");
    //telemetry.update();
    int startPos = 0;
    
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    
    
   // RoboCam camera1 = new RoboCam(myAprilTagProcessor,myVisionPortal,motors,this);
    //Launcher drone = new Launcher(launcherServo, elevationServo);
    //Claw claw = new Claw(clawServo);
    //Arm arm = new Arm(elbowServo,hardwareMap);
   // PidController moveX = new PidController(.0005,0.0001,.0005,.050,100);     
    
    
    
    PidController moveX = new PidController(.0002,0,.0001);
    waitForStart();
    
    imu.resetYaw();
    boolean found = false;
    int blueValue = 300;
    int redValue = 300;
    double powerSetting = 1.0;
   
    motors.move(.25,0,0);
    
   // sleep(1000);
    //motors.allStop();
    double x = 0;
    int setPoint = motors.getCurrentStepLateral()-8200; 
    moveX.setSetPoint(setPoint);
    int numZeros = 0;
    do{
      x = moveX.getPidOutput(motors.getCurrentStepLateral());
      motors.move(x,0,-.01);
      sleep(5);
      telemetry.addData("Error ", setPoint-motors.getCurrentStepLateral());
      telemetry.addData("P ",moveX.getP());
      telemetry.addData("I ",moveX.getI());
      telemetry.addData("D ",moveX.getD());
      telemetry.addData("X: ", x);
      telemetry.update();
      if(x==0){
       numZeros++;
      }
    }  while(x!=0&&opModeIsActive()&&numZeros<=3);
   motors.allStop();
      sleep(20000);
     
  }
    
  
    
}
