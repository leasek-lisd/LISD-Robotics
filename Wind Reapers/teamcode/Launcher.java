package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;


public class Launcher {
  
  
    // todo: write your code here
    private Servo launcherServo;
    private Servo lifterServo;
    
    public Launcher(Servo launchServo, Servo liftServo){
      launcherServo = launchServo;
      lifterServo = liftServo;
    }
    
    public void lift(){
      lifterServo.setPosition(.65);
    }
    
    public void shoot(){
      launcherServo.setPosition(0);
    }
    
}