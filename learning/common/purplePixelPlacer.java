public boolean findPropBehind(int startPos){
     /* startPos:
       1 - Blue / Nearest Backstage
       2 - Blue / Nearest Audience 
       3 - Red / Nearest Backstage
       4 - Red / Nearest Audience 
       
       Assuming left side mount
     */
     
     if(startPos==1||startPos ==4){
       strafeRightTimed(200,1);
       backward(.5);
       double pingDistance = 1;
       
       
       boolean atFirstLeg = false;
       boolean pastFirstLeg = false;
       boolean propFound = false;
       boolean skipLine = false;
       boolean firstLineFound = true;
       boolean colorMatch = false;
       
       String stateMsg = "Searching for "+legLength;
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
       allStop();
       telemetry.addData(">", "Stopped, prop found skipLine = "+skipLine);
         telemetry.update();
       sleep(1000);
       
       // Add something to verify stop in correct position
       propFound = false;
       strafeLeft(.5);
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
       allStop();
       //raise servo arm
       sleep(250);
       servo0.setPosition(servo0Up);
       strafeRightTimed(100,.5);
        telemetry.addData(">", "Sleeping" + skipLine);
         telemetry.update();
       sleep(5000);
       
     }
    
       
       
       return true;
   }
