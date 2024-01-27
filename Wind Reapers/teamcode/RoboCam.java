package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.ExecutionException;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import java.util.List;
import java.lang.*;

public class RoboCam {
  private AprilTagProcessor myAprilTagProcessor;
  private VisionPortal myVisionPortal;
  private DriveMotors motors;
  private double cameraOffset;
  private double cameraSetback;
  private LinearOpMode opMode;
  private double targetX = 0;
  private double targetRange = 0;
  private double targetYaw = 0;
  
  
  public RoboCam(AprilTagProcessor inMyAprilTagProcessor,VisionPortal inMyVisionPortal,DriveMotors inMotors, LinearOpMode inOpMode){
    myAprilTagProcessor = inMyAprilTagProcessor;
    myVisionPortal = inMyVisionPortal;
    motors = inMotors;
    cameraOffset = 0;
    cameraSetback = 3;
    opMode = inOpMode;
  }
    // todo: write your code here
    public void telemetryAprilTag(LinearOpMode opMode) throws InterruptedException, ExecutionException {
    Telemetry telemetry = opMode.telemetry;
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;

    // Get a list of AprilTag detections.
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
      telemetry.addLine("");
      if (myAprilTagDetection.metadata != null) {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
        telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
        telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
        telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
      } else {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
        telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
      }
    }
    telemetry.addLine("");
    telemetry.addLine("key:");
    telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    telemetry.addLine("RBE = Range, Bearing & Elevation");

  }
  
  public boolean lineUp(int tagID, LinearOpMode opMode) //throws InterruptedException, ExecutionException 
  {
    Telemetry telemetry = opMode.telemetry;
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;
    
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
       myAprilTagDetection = myAprilTagDetection_item;
       telemetry.addLine("");
       if (myAprilTagDetection.id == tagID ) {
         boolean onTrack = false;
         telemetry.addLine("x" + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1));
         if (myAprilTagDetection.ftcPose.x-cameraOffset > 1) {
           motors.rightInches(myAprilTagDetection.ftcPose.x);
           telemetry.addLine("Sliding Right -");
           telemetry.addLine("Move Right" + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1));
         } 
         else if (myAprilTagDetection.ftcPose.x + cameraOffset < -1) {
           motors.leftInches(-myAprilTagDetection.ftcPose.x);
           telemetry.addLine("Sliding left -");
          telemetry.addLine("Move Left" + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1));
         } 
         else {
          onTrack = true;
          telemetry.addLine("On Track" + "");
         }
      
      return true;  
      }
      
    }
    return false;
  }
  
  public void squareUp(int tagID, LinearOpMode opMode) throws InterruptedException, ExecutionException {
    Telemetry telemetry = opMode.telemetry;
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;
    // Get a list of AprilTag detections.
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
      telemetry.addLine("");
      if (myAprilTagDetection.id == tagID ) {
      
        if (myAprilTagDetection.ftcPose.yaw > .5) {
          telemetry.addLine("Sliding Rear -");
        //  motors.slideRear(-1);
          telemetry.addLine("Move Right" + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1));
        } else if (myAprilTagDetection.ftcPose.yaw < -.5) {
          telemetry.addLine("Sliding Rear +");
          motors.slideFront(-1);
          
          telemetry.addLine("Move Left" + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1));
        } else {
          telemetry.addLine("On Track" + "");
          motors.allStop();
        }
        telemetry.addLine("yaw: "+myAprilTagDetection.ftcPose.yaw);
        telemetry.update();
      }
      else{telemetry.addLine("Tag "+tagID+" not detected");
        telemetry.update();
      }
    }
  }
  
  public boolean approach(int tagID, LinearOpMode opMode) throws InterruptedException, ExecutionException {
    Telemetry telemetry = opMode.telemetry;
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;
    getTagYaw(tagID);
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
       myAprilTagDetection = myAprilTagDetection_item;
       telemetry.addLine("");
       if (myAprilTagDetection.id == tagID ) {
         boolean onTrack = false;
         telemetry.addLine("x" + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1));
         double moveAmount = myAprilTagDetection.ftcPose.y-6.0;
         motors.forwardInches(moveAmount) ;
         return true;
        }
    }
    return false;
  }
  
  public boolean tagIsVisible(int tagID, LinearOpMode opMode) throws InterruptedException, ExecutionException {
    Telemetry telemetry = opMode.telemetry;
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;
    
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
       myAprilTagDetection = myAprilTagDetection_item;
       telemetry.addLine("");
       if (myAprilTagDetection.id == tagID ) {
          return true;
        }
    }
    return false;
  }
  
  
  
  public void getTagLocation(int tagID) throws InterruptedException, ExecutionException {
    Telemetry telemetry = opMode.telemetry;
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;
    // Get a list of AprilTag detections.
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
      
      if (myAprilTagDetection.id == tagID ) {
        targetX = myAprilTagDetection.ftcPose.x;
        targetYaw = myAprilTagDetection.ftcPose.yaw;
      }
    }
  //  return 0;
  }
  
  public double getTagX(int tagID) // throws InterruptedException, ExecutionException 
  {
    Telemetry telemetry = opMode.telemetry;
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;
    // Get a list of AprilTag detections.
    boolean isFound = false;
    while(!isFound){
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
      
      if (myAprilTagDetection.id == tagID ) {
        return myAprilTagDetection.ftcPose.range;
      }
    }}
    return 0;
  }
  
  public double getTagYaw(int tagID) 
  //throws InterruptedException, ExecutionException 
  {
    Telemetry telemetry = opMode.telemetry;
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;
    // Get a list of AprilTag detections.
    boolean isFound = false;
    while(!isFound){
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
      
      if (myAprilTagDetection.id == tagID ) {
        isFound = true;
        return myAprilTagDetection.ftcPose.yaw;
      }
    }
  }
    return 0;
  }
  
  public void lineUp2(int tagID){
  //  if(yaw<0){
   //   motors.rotateRightDegrees((int)-yaw,.25);
    //  getTagLocation(int tagID);
      
      double turnAmount = getTagYaw(tagID);
      //double moveDistance = Math.abs(Math.sin(turnAmount)*180/(3.1416))*getTagX(tagID);
      double moveDistance = getTagX(tagID)*Math.sin(toRadians(90-turnAmount))/Math.sin(toRadians(90));
      
      //telemetry.addData("turn Amount: ", turnAmount);
      //telemetry.addData("Distance: ", moveDistance);
      //telemetry.update();
      
      motors.rotateRightDegrees((int)turnAmount,.8);
      motors.leftInches(moveDistance);
   // }
  }
  
  
  public void lineUp3(double x, double yaw){
  //  if(yaw<0){
   //   motors.rotateRightDegrees((int)-yaw,.25);
    //  getTagLocation(int tagID);
      
      double turnAmount = yaw;
      double moveDistance = Math.abs(Math.sin(turnAmount)*180/(3.1416))*x;
      
      motors.rightInches(moveDistance);
      motors.rotateRightDegrees((int)turnAmount,.8);
   // }
  }
  
  public double toRadians(double degrees){
    return degrees*3.1415/180;
  }
}
