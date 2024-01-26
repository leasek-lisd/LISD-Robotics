package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.ExecutionException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Plane 
{
    Servo launcher;
    
    public Plane(HardwareMap hardwareMap)
    {
        launcher = hardwareMap.get(Servo.class, "launcherC1");
        //launcher.setDirection(Servo.Direction.REVERSE);
        
    }
    public void fire()
    {
        launcher.setPosition(1);
    }
    public void reload()
    {
        launcher.setPosition(0.5);
    }
}