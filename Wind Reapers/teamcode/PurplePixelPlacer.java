package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.ExecutionException;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PurplePixelPlacer  
{
  double pingDistance = 1;
  double legLength = 20.0;
  double maxPropRange = 25.0;
  double minPropRange = 2.0;
  double closeLineMin = 2;
  double closeLineMax = 4;
  double midLineMax = 12;
  double farLineMax = 20;
 
    double colorUp = .7;
    // todo: write your code here
   // public void runOpMode() {
   public void init(LinearOpMode linearOpMode){

}   
  
 
 public int findProp(
     
     int startPos , 
     DriveMotors motors, 
     Servo servoLifterMotor,
     ColorSensor color,
     DistanceSensor distance1,
     LinearOpMode opMode
     )
     throws InterruptedException, ExecutionException
    {
     Telemetry telemetry = opMode.telemetry;
     /* startPos:
       1 - Blue / Nearest Backstage
       2 - Blue / Nearest Audience 
       3 - Red / Nearest Backstage
       4 - Red / Nearest Audience 
       
       Assuming forward mount, robot starting facing backstage
     */
    
       int randomLocation = 0;
       boolean atFirstLeg = false;
       boolean pastFirstLeg = false;
       boolean propFound = false;
       boolean skipLine = false;
       boolean firstLineFound = true;
       boolean colorMatch = false;
       servoLifterMotor.setPosition(0);
       String stateMsg = "Searching for "+legLength;
       motors.backwardInches(13);
       while(motors.areBusy()){
        telemetry.addLine("Backing Up");
        telemetry.update();
       }
       if(startPos==1||startPos ==2){  //blue
         telemetry.addLine("going Right");
         telemetry.update();
         motors.strafeRight(.5);}
       else {
         telemetry.addLine("going Left");
         telemetry.update();
         motors.strafeLeft(.5);
       }
       motors.strafeLeft(.5);
       ElapsedTime myElapsedTime2 = new ElapsedTime();
       ElapsedTime myTimer = new ElapsedTime();
       while(myElapsedTime2.seconds()<0){
        telemetry.addData("waiting: ",myElapsedTime2.seconds());
         telemetry.update();
       }
       
         while(!(atFirstLeg&&pastFirstLeg&propFound)&&opMode.opModeIsActive()&&myElapsedTime2.seconds() < 150) {
         
         pingDistance = distance1.getDistance(DistanceUnit.INCH);
         telemetry.addData(">", (stateMsg + " / " + pingDistance));
       
        
         telemetry.update();
       // accuracy can be improved with april tags but would require camera facing that direction 
       // also need to add something from a timeout in case it gets lost
        if(!atFirstLeg&&pingDistance <= legLength){
           atFirstLeg = true;
           stateMsg+="\nfirst leg";
           myElapsedTime2.reset();
         }
         if(!pastFirstLeg&&atFirstLeg&&pingDistance > legLength){
           pastFirstLeg = true;
           stateMsg+="\npast first leg";
           myElapsedTime2.reset();
         }
         if(atFirstLeg&&pastFirstLeg&&pingDistance <= farLineMax){
           propFound = true;
           stateMsg+="\nfound prop";
           myElapsedTime2.reset();
           if(pingDistance>midLineMax){
             firstLineFound = false;
             skipLine = true;
             stateMsg+="\n found long";
             randomLocation = 3;
           }
           else if(pingDistance>closeLineMax){
            stateMsg+="\n found mid";
            randomLocation = 2;
           }
           else { 
             stateMsg+="\nfound short";
             randomLocation = 1;
           }
         }
        }
        
        
          motors.allStop();
       telemetry.addData(">", "Stopped, prop found skipLine = "+skipLine+" "+randomLocation);
         telemetry.update();
      
       
       // Add something to verify stop in correct position
       propFound = false;
       motors.forward(.5);
       stateMsg = "searching for color "+skipLine+" / "+firstLineFound;
       boolean passedFirstLine = false;
       while(!propFound&&opMode.opModeIsActive()){
         // need to add timeout
         // need to add "lost" logic
         
         colorMatch = 400 < color.blue() || 400 < color.red();
         
         if(skipLine&&colorMatch&&!passedFirstLine){
          // skipLine = false;
           firstLineFound = true;
           stateMsg += "found first line\n";
           Thread.sleep(1000);
         }
         
         if(skipLine&&firstLineFound&&!colorMatch&&!passedFirstLine){
           skipLine = false;
          // stateMsg += "passed first line\n";
          passedFirstLine = true;
           telemetry.addData(">", "passed first line");
           telemetry.update();
           Thread.sleep(1000);
         }
         
         if(!skipLine&&firstLineFound&&colorMatch){
           propFound = true;
           stateMsg += "found line\n";
         }
         telemetry.addData(">", stateMsg);
         telemetry.update();
       }
       motors.allStop();
       //raise servo arm
      
       servoLifterMotor.setPosition(colorUp);
       Thread.sleep(750);
       motors.backwardInches(2);
        telemetry.addData(">", "Sleeping" + skipLine);
         telemetry.update();
     
 return randomLocation;}

 
} 