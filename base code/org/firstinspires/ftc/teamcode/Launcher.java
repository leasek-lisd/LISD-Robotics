package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;


public class Launcher {
  
  private Servo launcherServo;
    // todo: write your code here
    
    public Launcher(Servo lServo){
      launcherServo = lServo;
    }
    
    public void shoot(){
      launcherServo.setPosition(0);
    }
    
}