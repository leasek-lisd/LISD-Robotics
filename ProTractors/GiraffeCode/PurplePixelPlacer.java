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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PurplePixelPlacer 
{
    ColorSensor csensor;
    DistanceSensor dsensor;
    Servo purple;
    public PurplePixelPlacer(HardwareMap hardwareMap)
    {
       // purple = hardwareMap.get(Servo.class, "purple");
       // dsensor = hardwareMap.get(DistanceSensor.class, "DistanceSensorC2");
       // csensor = hardwareMap.get(ColorSensor.class, "ColorSensorC1");
    }
    public double checkDistance()
    {
      return dsensor.getDistance(DistanceUnit.INCH); 
    }
    public double checkBlue()
    {
      return csensor.blue();
    }
    public double checkRed()
    {
      return csensor.red();
    }
    public void hammerUp()
    {
        purple.setPosition(0);
    }
    public void hammerDown()
    {
        purple.setPosition(0.35);
    }
}