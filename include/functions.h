#include "main.h"
#include "string.h"
#include "math.h"

TaskHandle ultTask;
bool switchPressed;
bool running;

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
	setFullPower(4, direction);
	setFullPower(5, direction);
	setFullPower(6, direction);
	setFullPower(7, direction);
}

void stopCatapultMotors() {
	motorStop(4);
	motorStop(5);
	motorStop(6);
	motorStop(7);
}

void setCatapultMotors() {
	// Controlled by second joystick
	if (joystickGetDigital(1, 6, JOY_DOWN)) {
		setCatapultMotorsToFull(true);
	} else if (joystickGetDigital(1, 6, JOY_UP)) {
		setCatapultMotorsToFull(false);
	} else {
		if (!running) {
			stopCatapultMotors();
		}
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
	//	taskResume(ultTask);
	//	int ultVal = 500;
	while (abs(diff) < targetDiff) {
		//		ultVal = getUltVal();
		potVal = analogRead(1);
		diff = initialPotVal - potVal;
		setFullPower(4, false);
		setFullPower(5, false);
		setFullPower(6, false);
		setFullPower(7, false);
		lcdPrint(uart1, 2, "%d", diff);
		delay(20);
	}
	running = false;
	//	taskSuspend(ultTask);
}

void launchCatapult(int length) {
	setFullPower(4, false);
	setFullPower(5, false);
	setFullPower(6, false);
	setFullPower(7, false);
	delay(length);
}

void checkCatapult(void *ignore) {
	while (1) {
		if (joystickGetDigital(1, 8, JOY_DOWN)) {
			// Pull back catapult and prepare for launch
			pullCatapultBack(2875);

			// Push back on motors to resist elasticity
			int motorResistance = -20;
			motorSet(4, motorResistance);
			motorSet(5, motorResistance);
			motorSet(6, motorResistance);
			motorSet(7, motorResistance);
		}
		if (joystickGetDigital(1, 8, JOY_DOWN)) {
			launchCatapult(MAX_LAUNCH_VAL);
			stopCatapultMotors();
		}
		taskDelay(20);
	}
}

void checkForIndiv() {
	if (!joystickGetDigital(1, 8, JOY_UP)) {
		if (joystickGetDigital(1, 7, JOY_UP)) {
			setFullPower(8, true);
		} else if (joystickGetDigital(1, 7, JOY_DOWN)) {
			setFullPower(8, false);
		} else if (!(joystickGetDigital(1, 5, JOY_UP)
				|| joystickGetDigital(1, 5, JOY_DOWN))) {
			motorStop(8);
		}
	} else {
		if (joystickGetDigital(1, 7, JOY_UP)) {
			setFullPower(INTAKES, true);
		} else if (joystickGetDigital(1, 7, JOY_DOWN)) {
			setFullPower(INTAKES, false);
		} else if (!(joystickGetDigital(1, 5, JOY_UP)
				|| joystickGetDigital(1, 5, JOY_DOWN))) {
			motorStop(INTAKES);
		}
	}
}

void checkForManualDrive() {
	if (joystickGetAnalog(1, 3) == 0 && joystickGetAnalog(1, 2) == 0) {
		if (joystickGetDigital(1, 7, JOY_UP)) {
			motorSet(1, 127 * MULTIPLIER);
			motorSet(-1, 127);
			motorSet(-1, 127);
			motorSet(-10, 127 * MULTIPLIER);
		} else if (joystickGetDigital(1, 7, JOY_DOWN)) {
			motorSet(1, 127 * MULTIPLIER);
			motorSet(2, -127);
			motorSet(9, -127);
			motorSet(10, -127 * MULTIPLIER);
		} else {
			motorStop(1);
			motorStop(2);
			motorStop(9);
			motorStop(10);
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
void runTower(int time, void *ignore) {
	int delay;
	switch (time) {
	case 0:
		delay = 500;
		break;
	case 1:
		delay = 1000;
		break;
	case 2:
		delay = 1500;
		break;
	default:
		delay = 1000;
		break;
	}
	setFullPower(TOWER, false);
	taskDelay(delay);
	motorStop(TOWER);
}

void pullBackAndLaunch() {
	// Launch loaded ball first; will be in the pulled back position initially
	launchCatapult(MAX_LAUNCH_VAL);
	stopCatapultMotors();
	// Pull back catapult using adjusted potentiometer values
	for (int i = 0; i < 2; i++) {
		// Run to bring in preloaded balls
		TaskHandle towTask = taskCreate(runTower, TASK_DEFAULT_STACK_SIZE, &i,
						TASK_PRIORITY_DEFAULT);
		pullCatapultBack(2975);
		// Stop motors after pull back & wait for balls to move into catapult hand
		stopCatapultMotors();
		delay(500);
		// Launch & repeat
		launchCatapult(MAX_LAUNCH_VAL);
		stopCatapultMotors();
		taskDelete(towTask);
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
	int masterError, masterPreviousError, masterIntegral, masterDerivative,
			masterOutput;
	int startTime = millis();

	float kP_slave, kI_slave, kD_slave;
	int slaveError, slavePreviousError, slaveIntegral, slaveDerivative,
			slaveOutput, slaveFinal;

	kP_master = .15;
	kI_master = .000001;
	kD_master = .2;
	kP_slave = .1;
	kI_slave = 0;
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

		slaveOutput = (slaveError * kP_slave) + (slaveIntegral * kI_slave)
				+ (slaveDerivative * kD_slave);
		slaveFinal = masterOutput + slaveOutput;

		lcdPrint(uart1, 1, "M: %d, %d", masterOutput, averageVal);
		lcdPrint(uart1, 2, "S: %d", slaveFinal);
		lcdPrint(uart2, 1, "R: %d", rightVal);
		lcdPrint(uart2, 2, "L: %d", leftVal);

		// Left motor master
		motorSet(2, -masterOutput);
		motorSet(9, -masterOutput);

		// Right motor slave
		motorSet(1, slaveFinal);
		motorSet(10, -slaveFinal);

		delay(20);
	}
}

void checkForward() {
	// Reset encoder values before moving
	imeReset(0);
	imeReset(1);

	driveStraight(2200, 5000);

	pullBackAndLaunch();
}

void moveAuto() {
	// Reset encoder values before moving
	imeReset(0);
	imeReset(1);

	driveStraight(1100, 3000);

	pullBackAndLaunch();
}

void checkLimSwitch() {
	// Insurance
	delay(3000);
	if (!switchPressed) {
		motorStopAll();
	}
}
