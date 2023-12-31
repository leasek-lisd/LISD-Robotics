// should be in first block uner public class....
ColorSensor color1;
DistanceSensor distance1;

// Should be under public void runOpMode()
color1 = hardwareMap.get(ColorSensor.class, "color1");
distance1 = hardwareMap.get(DistanceSensor.class, "distance1");

// Add at begining of code
motorLeft.setDirection(DcMotor.Direction.REVERSE);

// Add after Main method (between the last two curly brackets }
public void forward(int runTime){
	motorLeft.setPower(1);
	motorRight.setPower(1);
	sleep(runTime);
	motorLeft.setPower(0);
	motorRight.setPower(0);
	sleep(100);
}

public void backward(int runTime){
	motorLeft.setPower(-1);
	motorRight.setPower(-1);
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

public void forwardSteps(int steps){
	int targetPos = leftPos + steps * stepPos;
	leftPos = targetPos;
	rightPos = targetPos;
	motorLeft.setTargetPosition(targetPos);
	motorRight.setTargetPosition(targetPos);
	waitForStart();
	motorLeft.setPower(1);
	motorRight.setPower(1);
	sleep(1000);
}
public void rotateRightSteps(int steps){
	leftPos += steps;
	rightPos -= steps;
	motorLeft.setTargetPosition(leftPos);
	motorRight.setTargetPosition(rightPos);
	waitForStart();
	motorLeft.setPower(.5);
	motorRight.setPower(.5);
	sleep(2000);
}

}
