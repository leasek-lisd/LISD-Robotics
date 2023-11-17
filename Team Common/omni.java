public void omni(double x, double y, double turn){
    
        double rawTheta = Math.atan2(y,x);
        double theta = rawTheta -Math.PI/4;
        double leftFront = 0;
        double rightFront = 0;
        double leftRear = 0;
        double rightRear = 0;
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        double max = Math.max(Math.abs(sin),Math.abs(cos));
      
        double power = Math.sqrt(x*x+y*y);
        
        leftFront = power * cos/max + turn;
        rightFront = power * sin/max - turn;
        leftRear = power * cos/max - turn;
        rightRear = power * sin/max + turn;
        
        if ((power + Math.abs(turn)) > 1) {
          leftFront   /= power + Math.abs(turn);
          rightFront /= power + Math.abs(turn);
          leftRear    /= power + Math.abs(turn);
          rightRear  /= power + Math.abs(turn);
        }
        frontLeftDrive.setPower(leftFront);
      frontRightDrive.setPower(rightFront);
      backLeftDrive.setPower(leftRear);
      backRightDrive.setPower(rightRear);

        
    }
    
    public static double roundMe(double num){
        return Math.round(num*100)/100.0;
    }
