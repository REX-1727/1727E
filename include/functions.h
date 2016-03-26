#include "main.h"
#include "string.h"
#include "math.h"

TaskHandle ultTask;
Gyro gy;
bool switchPressed, running, pulledBack = false, reversed = false, firstCheck = true;
int time = 0, numLaunches = 0;

void checkDrive() {
	//	 Set motor values to joystick values
	int joyLeftVal = -joystickGetAnalog(1, 3), joyRightVal = joystickGetAnalog(1, 2);

	if(joystickGetDigital(1, 7, JOY_UP)) {
		reversed = false;
	} else if (joystickGetDigital(1, 7, JOY_DOWN)) {
		reversed = true;
	}

	if(!reversed) {
		motorSet(FRONT_LEFT, joyLeftVal);
		motorSet(BACK_LEFT, joyLeftVal);

		motorSet(FRONT_RIGHT, joyRightVal);
		motorSet(BACK_RIGHT, -joyRightVal);
	} else {
		motorSet(FRONT_LEFT, joyRightVal);
		motorSet(BACK_LEFT, joyRightVal);

		motorSet(FRONT_RIGHT, joyLeftVal);
		motorSet(BACK_RIGHT, -joyLeftVal);
	}
}

void setFullPower(int port, bool direction) {
	int power = 127;
	if (!direction) {
		power *= -1;
	}
	motorSet(port, power);
}

void setTowerAndIntake() {
	if (joystickGetDigital(1, 5, JOY_DOWN)) {
		setFullPower(INTAKES, true);
		setFullPower(TOWER, true);
	} else if (joystickGetDigital(1, 5, JOY_UP)) {
		setFullPower(INTAKES, false);
		setFullPower(TOWER, false);
	} else {
		motorStop(INTAKES);
		motorStop(TOWER);
	}
}

// Catapult functions

void setCatapultMotorsToFull(bool direction) {
	setFullPower(CAT_1, direction);
	setFullPower(CAT_2, direction);
	setFullPower(CAT_3, direction);
	setFullPower(CAT_4, direction);
}

void runCatapultMotors(int power) {
	motorSet(CAT_1, power);
	motorSet(CAT_2, power);
	motorSet(CAT_3, power);
	motorSet(CAT_4, power);
}

void stopCatapultMotors() {
	motorStop(CAT_1);
	motorStop(CAT_2);
	motorStop(CAT_3);
	motorStop(CAT_4);
}

void catapultResist() {
	// Push back on motors to resist elasticity
	motorSet(CAT_1, MOTOR_RESISTANCE);
	motorSet(CAT_2, MOTOR_RESISTANCE);
	motorSet(CAT_3, MOTOR_RESISTANCE);
	motorSet(CAT_4, MOTOR_RESISTANCE);
}

void setCatapultMotors() {
	if (joystickGetDigital(1, 6, JOY_DOWN)) {
		setCatapultMotorsToFull(true);
		pulledBack = false;
		firstCheck = true;
	} else if (joystickGetDigital(1, 6, JOY_UP)) {
		setCatapultMotorsToFull(false);
		pulledBack = false;
		firstCheck = true;
	} else if (!running && !pulledBack && firstCheck) {
		// Hold down button on release to trigger resistance
		if(joystickGetDigital(1, 7, JOY_LEFT)) {
			catapultResist();
		} else {
			stopCatapultMotors();
		}
		firstCheck = false;
	}
}

void playSound() {
	speakerPlayRtttl(
			"Eyeofthe:d=4,o=5,b=112:8d, 8e, 8f, 8p, 8f, 16f, 8f, 16p, 8e, 8d, 8c, 8c, 8d, 8e, 8d, 8p, 8d, 8e, 8f, 16p, 32p, 8e, 8f, 8g, 16p, 32p, 8f, 8g, 2a, 4p, 8d, 16c, 8d, 16p, 8c");
}

void checkUlt(void *ignore) {
	Ultrasonic ult = ultrasonicInit(12, 1);
	while (1) {
		int ultVal = ultrasonicGet(ult);
		lcdPrint(uart1, 2, "Ult: %d", ultVal);
		if (ultVal != 0 && ultVal < 40) {
			stopCatapultMotors();
		}
		taskDelay(200);
	}
}

int getUltVal() {
	Ultrasonic ult = ultrasonicInit(12, 1);
	int ultVal = ultrasonicGet(ult);
	lcdPrint(uart1, 2, "Ult: %d", ultVal);
	ultrasonicShutdown(ult);
	return ultVal;
}

void pullCatapultBack(int targetDiff) {
	running = true;
	int diff = 0;
	int potVal;
	int initialPotVal = analogRead(1);
	int timeElapsed = millis();
	while ((abs(diff) < targetDiff) && ((millis() - timeElapsed) <= 1500)) {
		potVal = analogRead(1);
		diff = initialPotVal - potVal;
		if(abs(diff) < targetDiff * 0.75) {
			setCatapultMotorsToFull(false);
		} else {
			runCatapultMotors(CAT_PARTIAL_POWER);
		}
		lcdPrint(uart1, 2, "%d", diff);
		delay(20);
	}
	running = false;
	pulledBack = true;
}

void launchCatapult(int length) {
	setFullPower(CAT_1, false);
	setFullPower(CAT_2, false);
	setFullPower(CAT_3, false);
	setFullPower(CAT_4, false);
	delay(length);
	pulledBack = false;
}

void checkCatapult(void *ignore) {
	while (1) {
		if (joystickGetDigital(1, 8, JOY_DOWN) && !pulledBack) { // Currently pulled back
			// Pull back catapult and prepare for launch
			pullCatapultBack(POT_MAX_DIFF);
			catapultResist();
		} else if (joystickGetDigital(1, 8, JOY_DOWN)) {
			launchCatapult(MAX_LAUNCH_VAL);
			stopCatapultMotors();
		} else if(joystickGetDigital(1, 8, JOY_UP)) {
			for(int i = 1; i < NUM_AUTO_LAUNCHES; i++) {
				if(joystickGetDigital(1, 6, JOY_DOWN)) {
					break;
				} else {
					pullCatapultBack(POT_MAX_DIFF);
					catapultResist();
					delay(750);
					launchCatapult(MAX_LAUNCH_VAL);
					stopCatapultMotors();
					setFullPower(TOWER, true);
					delay(750);
					motorStop(TOWER);
				}
			}
		}
		taskDelay(20);
	}
}

void checkForManualDrive() {
	// Don't use setFullPower() methods for easy adjustment
	if (joystickGetAnalog(1, 3) == 0 && joystickGetAnalog(1, 2) == 0) {
		if (joystickGetDigital(1, 7, JOY_UP)) {
			motorSet(FRONT_RIGHT, 127 * MULTIPLIER);
			motorSet(-FRONT_RIGHT, 127);
			motorSet(-FRONT_RIGHT, 127);
			motorSet(-BACK_RIGHT, 127 * MULTIPLIER);
		} else if (joystickGetDigital(1, 7, JOY_DOWN)) {
			motorSet(FRONT_RIGHT, 127 * MULTIPLIER);
			motorSet(FRONT_LEFT, -127);
			motorSet(BACK_LEFT, -127);
			motorSet(BACK_RIGHT, -127 * MULTIPLIER);
		} else {
			motorStop(FRONT_RIGHT);
			motorStop(FRONT_LEFT);
			motorStop(BACK_LEFT);
			motorStop(BACK_RIGHT);
		}
	}
}

// LCD functions

bool checkBattery(int *lcd) {
	bool backlight = false; // Off by default
	if (powerLevelMain() == 0) { // Turn on backlight if powered by computer
		backlight = true;
	}
	lcdSetBacklight(lcd, backlight);
	return backlight;
}

bool checkBacklight(int *lcd, bool backlight) {
	if (lcdReadButtons(lcd) == 1) {
		// Button 1 is pressed
		// Toggle backlight
		backlight = !backlight;
		lcdSetBacklight(lcd, backlight);
		// Delay for a half second to allow time to react
		delay(500);
	}
	return backlight;
}

void showBatteryOnLcd(int *lcd) {
	int power = powerLevelMain();
	if (power == 0) {
		// Powered by computer (Cortex battery is turned off or not present)
		lcdSetText(lcd, 1, "Main power off");
	} else {
		lcdPrint(lcd, 1, "Power: %d mV", power);
	}
}

void showPotVals(int *lcd, int port) {
	// Grab potentiometer value and print to LCD
	int potVal = analogRead(port);
	lcdPrint(lcd, 1, "Potentiometer");
	lcdPrint(lcd, 2, "value: %d", potVal);
}

void checkVel() {
	// Time period in milliseconds
	int timePeriod = 20;
	Encoder enc = encoderInit(3, 4, false);
	int ticks = 0;
	while (1) {
		// Read from potentiometer
		int newTicks = encoderGet(enc);
		int diff = newTicks - ticks;
		ticks = newTicks;

		int vel = diff / (timePeriod / 1000.0);

		lcdPrint(uart1, 2, "Vel: %d deg/s", vel);
		taskDelay(timePeriod);
	}
}

bool *initLcdVals() {
	bool newBacklight = checkBattery(uart1 );
	bool newSecondBacklight = checkBattery(uart2 );
	bool backlights[] = { newBacklight, newSecondBacklight };

	bool *backArr = malloc(sizeof(backlights));
	backArr[0] = backlights[0];
	backArr[1] = backlights[1];
	return backArr;
}

// AUTON FUNCTIONS
void runTower(void *ignore) {
	int delay;
	switch (time) {
	case 0:
		delay = 750;
		break;
	case 1:
		delay = 1750;
		break;
	case 2:
		delay = 2000;
		break;
	case 3:
		delay = 100;
		break;
	default:
		delay = 1500;
		break;
	}
	setFullPower(TOWER, false);
	taskDelay(delay);
	motorStop(TOWER);
	time++;
}

void delayLaunch(int time) {
	int del;
	switch(time) {
		case 0:
			del = 1250;
			break;
		case 1:
			del = 1250;
			break;
		case 2:
			del = 1500;
			break;
		case 3:
			del = 1000;
			break;
		default:
			del = 1250;
			break;
	}
	delay(del);
}

void pullBackAndLaunch() {
	// Launch loaded ball first; will be in the pulled back position initially
	launchCatapult(MAX_LAUNCH_VAL);
	stopCatapultMotors();
	// Pull back catapult using adjusted potentiometer values
	time = 0;
	for (int i = 0; i < 3; i++) {
		// Run tower to bring in last preloaded ball
		taskCreate(runTower, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
		pullCatapultBack(POT_MAX_DIFF);
		// Stop motors after pull back & wait for balls to move into catapult hand
		stopCatapultMotors();
		catapultResist();
		delayLaunch(i);
		// Launch & repeat
		launchCatapult(MAX_LAUNCH_VAL);
		stopCatapultMotors();
		// Deeper in the intakes needs a bit of delay
		if(i == 1 || i == 2) {
			delay(650);
		}
	}
}

void driveStraight(int target, int time) {
	// Left side: IME0, MOTOR9
	// Right side: IME1, MOTOR10

	// Reinit gyro
	gy = gyroInit(3, 0);

	// PID loop to drive straight
	int rightVal = 0;
	int leftVal = 0;
	int averageVal = 0;
	int rightVel = 0;
	int leftVel = 0;
	float kP_master, kI_master, kD_master;
	float masterError, masterPreviousError, masterIntegral, masterDerivative,
	masterOutput;
	int startTime = millis();

	float kP_differential, kI_differential, kD_differential;
	float differentialError, differentialPreviousError, differentialIntegral, differentialDerivative,
	differentialOutput;

	float leftFinal = 0, rightFinal = 0;
	float leftRatio = 1, rightRatio = 1;

	kP_master = .15;
	kI_master = .00000;
	kD_master = .2;
	kP_differential = .05;
	kI_differential = .000;
	kD_differential = 25;
	masterPreviousError = 0;
	differentialPreviousError = 0;
	while (millis() < startTime + time) {
		imeGet(0, &leftVal);
		imeGet(1, &rightVal);
		imeGetVelocity(0, &leftVel);
		imeGetVelocity(1, &rightVel);
		rightVal = abs(rightVal);
		leftVal = abs(leftVal);

		averageVal = (leftVal + rightVal) / 2;
		masterError = target - averageVal;
		masterIntegral += masterPreviousError;
		masterDerivative = masterError - masterPreviousError;
		if (masterError == 0)
			masterIntegral = 0;
		if (masterIntegral > 100)
			masterIntegral = 100;

		masterOutput = (masterError * kP_master) + (masterIntegral * kI_master)
						+ (masterDerivative * kD_master);

		if (masterOutput > 100){
			masterOutput = 100;
		}

		if(masterError > 360) {
			differentialError = -gyroGet(gy);
			differentialIntegral += differentialPreviousError;
			differentialDerivative = differentialError - differentialPreviousError;
			if (differentialError == 0)
				differentialIntegral = 0;
			if (differentialIntegral > 100)
				differentialIntegral = 100;


			differentialOutput = (differentialError * kP_differential) + (differentialIntegral * kI_differential)
						+ (differentialDerivative * kD_differential);

			rightFinal = masterOutput + differentialOutput;
			leftFinal = masterOutput - differentialOutput;

			lcdPrint(uart1, 1, "M: %f, %d", masterOutput, averageVal);
			lcdPrint(uart2, 1, "R: %d", rightVal);
			lcdPrint(uart2, 2, "L: %d", leftVal);

			rightRatio = rightFinal / masterOutput;
			leftRatio = leftFinal / masterOutput;
		} else {
			rightFinal = masterOutput * rightRatio;
			leftFinal = masterOutput * leftRatio;
		}

		// Left motor master
		motorSet(FRONT_LEFT, -leftFinal);
		motorSet(BACK_LEFT, -leftFinal);

		// Right motor slave
		motorSet(FRONT_RIGHT, rightFinal);
		motorSet(BACK_RIGHT, -rightFinal);

		delay(20);
	}
}

void checkForward() {
	// Reset encoder values before moving
	imeReset(0);
	imeReset(1);

	driveStraight(2240, 3500);

	setFullPower(FRONT_LEFT, true);
	setFullPower(BACK_LEFT, true);

	delay(150);

	motorStop(FRONT_LEFT);
	motorStop(BACK_LEFT);

	pullBackAndLaunch();
}

void moveAuto() {
	pullBackAndLaunch();
}
