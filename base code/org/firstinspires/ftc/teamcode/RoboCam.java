package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.ExecutionException;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import java.util.List;


public class RoboCam {
  private AprilTagProcessor myAprilTagProcessor;
  private VisionPortal myVisionPortal;
  private DriveMotors motors;
  private double cameraOffset;
  public RoboCam(AprilTagProcessor inMyAprilTagProcessor,VisionPortal inMyVisionPortal,DriveMotors inMotors){
    myAprilTagProcessor = inMyAprilTagProcessor;
    myVisionPortal = inMyVisionPortal;
    motors = inMotors;
    cameraOffset = 0;
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
  
  public void lineUp(LinearOpMode opMode) throws InterruptedException, ExecutionException {
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
      if (myAprilTagDetection.id == 9 ) {
      boolean onTrack = false;
      
        if (myAprilTagDetection.ftcPose.x-cameraOffset > 1) {
          motors.strafeRightTimed(.25,200);
          telemetry.addLine("Sliding Rear -");
          
          telemetry.addLine("Move Right" + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1));
        } else if (myAprilTagDetection.ftcPose.x + cameraOffset < -1) {
          motors.strafeLeftTimed(.25,200);
          telemetry.addLine("Sliding Rear -");
          
          telemetry.addLine("Move Left" + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1));
        } else {
          onTrack = true;
          telemetry.addLine("On Track" + "");
        }
      
        
      }
    }
  }
  
  public void squareUp(LinearOpMode opMode) throws InterruptedException, ExecutionException {
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
      if (myAprilTagDetection.id == 9 ) {
      
        if (myAprilTagDetection.ftcPose.yaw > 1) {
          telemetry.addLine("Sliding Rear -");
          motors.slideFront(-1);
          telemetry.addLine("Move Right" + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1));
        } else if (myAprilTagDetection.ftcPose.yaw < -1) {
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
    }
  }
}
