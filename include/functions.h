#include "main.h"
#include "string.h"
#include "math.h"

TaskHandle ultTask;

void setFullPower(int port, bool direction) {
	int power = 127;
	if (!direction) {
		power *= -1;
	}
	motorSet(port, power);
}

void setTowerAndIntake() {
	if (joystickGetDigital(1, 5, JOY_DOWN)) {
		setFullPower(3, true);
		setFullPower(8, true);
	} else if (joystickGetDigital(1, 5, JOY_UP)) {
		setFullPower(3, false);
		setFullPower(8, false);
	} else {
		motorStop(3);
		motorStop(8);
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
	if (joystickGetDigital(1, 6, JOY_DOWN)) {
		setCatapultMotorsToFull(true);
	} else if (joystickGetDigital(1, 6, JOY_UP)) {
		setCatapultMotorsToFull(false);
	} else {
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

void pullCatapultBack(int potVal, int newPotVal) {
	int diff = potVal - newPotVal;
//	taskResume(ultTask);
//	int ultVal = 500;
	while (abs(diff) < 700) {
//		ultVal = getUltVal();
		newPotVal = analogRead(1);
		diff = potVal - newPotVal;
		setFullPower(4, false);
		setFullPower(5, false);
		setFullPower(6, false);
		setFullPower(7, false);
		delay(20);
	}
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
	int potVal, newPotVal;
	while (1) {
		if (joystickGetDigital(1, 8, JOY_DOWN)) {
			potVal = analogRead(1);
			newPotVal = potVal;
			// Pull back catapult and prepare for launch
			pullCatapultBack(potVal, newPotVal);

			// Push back on motors to resist elasticity
			int motorResistance = -10;
			motorSet(4, motorResistance);
			motorSet(5, motorResistance);
			motorSet(6, motorResistance);
			motorSet(7, motorResistance);
		}
		if (joystickGetDigital(1, 8, JOY_DOWN)) {
			launchCatapult(750);
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
			setFullPower(3, true);
		} else if (joystickGetDigital(1, 7, JOY_DOWN)) {
			setFullPower(3, false);
		} else if (!(joystickGetDigital(1, 5, JOY_UP)
				|| joystickGetDigital(1, 5, JOY_DOWN))) {
			motorStop(3);
		}
	}
}

void pullBackAndLaunch() {
	// Launch loaded ball first; will be in the pulled back position initially
	launchCatapult(750);
	// Pull back catapult using adjusted potentiometer values
	for (int i = 0; i < 2; i++) {
		int potVal, newPotVal;
		potVal = analogRead(1);
		newPotVal = potVal;
		int diff = potVal - newPotVal;
		while (abs(diff) < 700) {
			newPotVal = analogRead(1);
			diff = potVal - newPotVal;
			setFullPower(4, false);
			setFullPower(5, false);
			setFullPower(6, false);
			setFullPower(7, false);
			delay(20);
		}
		// Stop motors after pull back & wait for balls to move into catapult hand
		stopCatapultMotors();
		delay(1500);
		// Launch & repeat
		launchCatapult(750);
		stopCatapultMotors();
	}
}

void driveStraight() {
	// Left side: IME0, MOTOR9
	// Right side: IME1, MOTOR10
	int leftWheel = 0, rightWheel = 0, resolution = 127;
	if (imeGet(0, &leftWheel) && imeGet(1, &rightWheel)) {
		int leftVel = 0, rightVel = 0;
		if (imeGetVelocity(0, &leftVel) && imeGetVelocity(1, &rightVel)) {
			if (leftVel > rightVel) {
				int newLeftPower = motorGet(9) + resolution, newRightPower =
						motorGet(10) - resolution;
				motorSet(1, newRightPower);
				motorSet(2, newLeftPower);
				motorSet(9, newLeftPower);
				motorSet(10, newRightPower);
			} else if (rightVel > leftVel) {
				int newLeftPower = motorGet(9) - resolution, newRightPower =
						motorGet(10) + resolution;
				motorSet(1, newRightPower);
				motorSet(2, newLeftPower);
				motorSet(9, newLeftPower);
				motorSet(10, newRightPower);
			}
		}
	}
}

void checkForward(void *ignore) {
	// Move forward until limit switch triggered
	// Right side runs fast; slow it down
	motorSet(1, -127);
	motorSet(2, 127);
	motorSet(9, 127);
	motorSet(10, 127);

	// Stop
	motorStop(1);
	motorStop(2);
	motorStop(9);
	motorStop(10);

	// Pull back catapult & launch appropriate # of times
	pullBackAndLaunch();
}

void checkForManualDrive() {
	if (joystickGetAnalog(1, 3) == 0 && joystickGetAnalog(1, 2) == 0) {
		if (joystickGetDigital(1, 7, JOY_UP)) {
			driveStraight();
		} else if (joystickGetDigital(1, 7, JOY_DOWN)) {
			motorSet(1, 127 * MULTIPLIER);
			motorSet(2, 127);
			motorSet(9, 127);
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

// Initialization functions

void initBot() {
	lcdInit(uart1 );
	lcdInit(uart2 );
	speakerInit();
	imeInitializeAll();

	taskCreate(playSound, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
//	ultTask = taskCreate(checkUlt, TASK_DEFAULT_STACK_SIZE, NULL,
//			TASK_PRIORITY_DEFAULT);
//	taskSuspend(ultTask);
	taskCreate(checkCatapult, TASK_DEFAULT_STACK_SIZE, NULL,
			TASK_PRIORITY_DEFAULT);
//	taskCreate(checkVel, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
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
