package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.ExecutionException;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PurplePixelPlacer  
{
 
    double colorUp = .7;
    // todo: write your code here
   // public void runOpMode() {
   public void init(LinearOpMode linearOpMode){

}
    public boolean findPropBehind(
     
     int startPos, 
     DriveMotors motors, 
     Servo servoLifterMotor,
     ColorSensor color,
     DistanceSensor distance1,
     LinearOpMode opMode)
     throws InterruptedException, ExecutionException
    {
     Telemetry telemetry = opMode.telemetry;
     /* startPos:
       1 - Blue / Nearest Backstage
       2 - Blue / Nearest Audience 
       3 - Red / Nearest Backstage
       4 - Red / Nearest Audience 
       
       Assuming left side mount
     */
    
     if(startPos==1||startPos ==4){
       
       
       double pingDistance = 1;
       double legLength = 20.0;
       double maxPropRange = 25.0;
       double minPropRange = 2.0;
       
       boolean atFirstLeg = false;
       boolean pastFirstLeg = false;
       boolean propFound = false;
       boolean skipLine = false;
       boolean firstLineFound = true;
       boolean colorMatch = false;
       servoLifterMotor.setPosition(0);
       String stateMsg = "Searching for "+legLength;
       
       motors.strafeRightTimed(200,1);
       motors.backward(.5);
       
       while(!(atFirstLeg&&pastFirstLeg&propFound)) {
         
         pingDistance = distance1.getDistance(DistanceUnit.INCH);
         telemetry.addData(">", (stateMsg + " / " + pingDistance));
         telemetry.update();
       // accuracy can be improved with april tags but would require camera facing that direction 
       // also need to add something from a timeout in case it gets lost
         if(!atFirstLeg&&pingDistance <= legLength){
           atFirstLeg = true;
           stateMsg+="\nfirst leg";
         }
         if(!pastFirstLeg&&atFirstLeg&&pingDistance > legLength){
           pastFirstLeg = true;
           stateMsg+="\npast first leg";
         }
         if(atFirstLeg&&pastFirstLeg&&pingDistance <= maxPropRange){
           propFound = true;
           stateMsg+="\nfound prop";
           if(pingDistance>minPropRange){
             firstLineFound = false;
             skipLine = true;
             stateMsg+="\npast found long";
           }
           else {
             stateMsg+="\nfound short";
           }
         }
       }
       motors.allStop();
       telemetry.addData(">", "Stopped, prop found skipLine = "+skipLine);
         telemetry.update();
      Thread.sleep(1000);
       
       // Add something to verify stop in correct position
       propFound = false;
       motors.strafeLeft(.5);
       stateMsg = "searching for color";
       while(!propFound){
         // need to add timeout
         // need to add "lost" logic
         
         colorMatch = 400 < color.blue() || 400 < color.red();
         
         if(skipLine&&colorMatch){
           skipLine = false;
           firstLineFound = true;
           stateMsg += "found first line\n";
         }
         
         if(!skipLine&&firstLineFound&&!colorMatch){
           skipLine = false;
          // stateMsg += "passed first line\n";
           telemetry.addData(">", "passed first line");
           telemetry.update();
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
      Thread.sleep(250);
       servoLifterMotor.setPosition(colorUp);
       motors.strafeRightTimed(.5,100);
        telemetry.addData(">", "Sleeping" + skipLine);
         telemetry.update();
      Thread.sleep(5000);
       
     }
 return true;}
 public boolean findProp(
     
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
       
       Assuming left side mount
     */
    
    // if(startPos==1||startPos ==4){
     
       boolean atFirstLeg = false;
       boolean pastFirstLeg = false;
       boolean propFound = false;
       boolean skipLine = false;
       boolean firstLineFound = true;
       boolean colorMatch = false;
       servoLifterMotor.setPosition(0);
       String stateMsg = "Searching for "+legLength;
       motors.backInches(13);
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
       //  telemetry.addData("key", myElapsedTime2.seconds());
        
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
         if(atFirstLeg&&pastFirstLeg&&pingDistance <= maxPropRange){
           propFound = true;
           stateMsg+="\nfound prop";
           myElapsedTime2.reset();
           if(pingDistance>minPropRange){
             firstLineFound = false;
             skipLine = true;
             stateMsg+="\npast found long";
           }
           else {
             stateMsg+="\nfound short";
           }
         }
        }
          motors.allStop();
       telemetry.addData(">", "Stopped, prop found skipLine = "+skipLine);
         telemetry.update();
      Thread.sleep(1000);
       
       // Add something to verify stop in correct position
       propFound = false;
       motors.strafeLeft(.5);
       stateMsg = "searching for color";
       while(!propFound&&opMode.opModeIsActive()){
         // need to add timeout
         // need to add "lost" logic
         
         colorMatch = 400 < color.blue() || 400 < color.red();
         
         if(skipLine&&colorMatch){
           skipLine = false;
           firstLineFound = true;
           stateMsg += "found first line\n";
         }
         
         if(!skipLine&&firstLineFound&&!colorMatch){
           skipLine = false;
          // stateMsg += "passed first line\n";
           telemetry.addData(">", "passed first line");
           telemetry.update();
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
       motors.backInches(2);
        telemetry.addData(">", "Sleeping" + skipLine);
         telemetry.update();
     // Thread.sleep(5000);
      
       
       
       
 //    }
 return true;}
  //  }
 
} 
