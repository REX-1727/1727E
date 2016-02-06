#include "main.h"
#include "string.h"
#include "math.h"

TaskHandle ultTask;
bool switchPressed, running, pulledBack = false, reversed = false;
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

void stopCatapultMotors() {
	motorStop(CAT_1);
	motorStop(CAT_2);
	motorStop(CAT_3);
	motorStop(CAT_4);
}

void setCatapultMotors() {
	if (joystickGetDigital(1, 6, JOY_DOWN)) {
		setCatapultMotorsToFull(true);
		pulledBack = false;
	} else if (joystickGetDigital(1, 6, JOY_UP)) {
		setCatapultMotorsToFull(false);
		pulledBack = false;
	} else if (!running && !pulledBack) {
			stopCatapultMotors();

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

/*
 * targetVal = 2875
 */
void pullCatapultBack(int targetDiff) {
	running = true;
	int diff = 0;
	int potVal;
	int initialPotVal = analogRead(1);
	int timeElapsed = millis();
	while ((abs(diff) < targetDiff) && ((millis() - timeElapsed) <= 1500)) {
		potVal = analogRead(1);
		diff = initialPotVal - potVal;
		setCatapultMotorsToFull(false);
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

void catapultResist() {
	// Push back on motors to resist elasticity
	motorSet(CAT_1, MOTOR_RESISTANCE);
	motorSet(CAT_2, MOTOR_RESISTANCE);
	motorSet(CAT_3, MOTOR_RESISTANCE);
	motorSet(CAT_4, MOTOR_RESISTANCE);
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
				pullCatapultBack(POT_MAX_DIFF);
				catapultResist();
				delay(750);
				launchCatapult(MAX_LAUNCH_VAL);
				stopCatapultMotors();
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
		delay = 0;
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

void pullBackAndLaunch() {
	// Launch loaded ball first; will be in the pulled back position initially
	launchCatapult(MAX_LAUNCH_VAL);
	stopCatapultMotors();
	// Pull back catapult using adjusted potentiometer values
	time = 0;
	for (int i = 0; i < 3; i++) {
		// Run tower to bring in last preloaded ball (fourth time only)
		taskCreate(runTower, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
		pullCatapultBack(POT_MAX_DIFF);
		// Stop motors after pull back & wait for balls to move into catapult hand
		stopCatapultMotors();
		catapultResist();
		delay(1250);
		// Launch & repeat
		launchCatapult(MAX_LAUNCH_VAL);
		stopCatapultMotors();
		// Deeper in the intakes needs a bit of delay
		if(i == 1 || i == 2) {
			delay(500);
		}
	}
}

void driveStraight(int target, int time) {
	// Left side: IME0, MOTOR9
	// Right side: IME1, MOTOR10

	// PID loop to drive straight
	int rightVal = 0;
	int leftVal = 0;
	int averageVal = 0;
	float kP_master, kI_master, kD_master;
	float masterError, masterPreviousError, masterIntegral, masterDerivative,
			masterOutput;
	int startTime = millis();

	float kP_slave, kI_slave, kD_slave;
	float slaveError, slavePreviousError, slaveIntegral, slaveDerivative,
			slaveOutput, slaveFinal;

	float slaveFactor;
	kP_master = .15;
	kI_master = .000001;
	kD_master = .2;
	kP_slave = .05;
	kI_slave = .0001;
	kD_slave = 0;
	masterPreviousError = 0;
	slavePreviousError = 0;
	while (millis() < startTime + time) {
		imeGet(0, &leftVal);
		imeGet(1, &rightVal);

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
				+ (masterDerivative * kD_master), 127;

		if (masterOutput > 127)
			masterOutput = 127;

		slaveError = leftVal - rightVal;
		slaveIntegral += slavePreviousError;
		slaveDerivative = slaveError - slavePreviousError;
		if (slaveError == 0)
			slaveIntegral = 0;
		if (slaveIntegral > 100)
			slaveIntegral = 100;

		slaveFactor = masterError/100;

		if(slaveFactor>1)
			slaveFactor = 1;

		if(slaveFactor<.1)
			slaveFactor = 0;

		slaveOutput = (slaveError * kP_slave) + (slaveIntegral * kI_slave)
				+ (slaveDerivative * kD_slave);


		slaveOutput = slaveOutput*(slaveFactor);

		slaveFinal = masterOutput + slaveOutput;

		lcdPrint(uart1, 1, "M: %f, %d", masterOutput, averageVal);
		lcdPrint(uart1, 2, "S: %f", slaveFinal);
		lcdPrint(uart2, 1, "R: %d", rightVal);
		lcdPrint(uart2, 2, "L: %d", leftVal);

		// Left motor master
		motorSet(FRONT_LEFT, -masterOutput);
		motorSet(BACK_LEFT, -masterOutput);

		// Right motor slave
		motorSet(FRONT_RIGHT, slaveFinal);
		motorSet(BACK_RIGHT, -slaveFinal);

		delay(20);
	}
}

void checkForward() {
	// Reset encoder values before moving
	imeReset(0);
	imeReset(1);

	driveStraight(2210, 4500);

	setFullPower(FRONT_LEFT, true);
	setFullPower(BACK_LEFT, true);
	delay(250);
	motorStop(FRONT_LEFT);
	motorStop(BACK_LEFT);

	pullBackAndLaunch();
}

void moveAuto() {
	pullBackAndLaunch();
}
