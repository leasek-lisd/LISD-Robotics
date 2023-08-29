// add to class
    ColorSensor color1;
    DistanceSensor distance1;

// Add to main
      color1 = hardwareMap.get(ColorSensor.class, "color1");
      distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
// Add after Main
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
        return distance1.getDistance(DiestanceUnit.COM);
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
