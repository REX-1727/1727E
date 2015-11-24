/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

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
void checkForward(void *ignore) {
	// Move forward until limit switch triggered
	// Right side runs fast; slow it down
	while (digitalRead(2)) {
		motorSet(1, -127 * MULTIPLIER);
		motorSet(2, -127);
		motorSet(9, -127);
		motorSet(10, 127 * MULTIPLIER);
	}

	// Stop
	motorStop(1);
	motorStop(2);
	motorStop(9);
	motorStop(10);

	// Pull back catapult & launc appropriate # of times
	pullBackAndLaunch();
}

void moveAuto() {
	// Set motors to proper speeds for 4 seconds
	motorSet(1, -127 * MULTIPLIER);
	motorSet(2, -127);
	motorSet(9, -127);
	motorSet(10, 127 * MULTIPLIER);
	delay(4000);

	motorStop(1);
	motorStop(2);
	motorStop(9);
	motorStop(10);

	pullBackAndLaunch();
}

/*
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */
void autonomous() {
	while (1) {
		lcdInit(uart1 );
		int controllerVal = analogRead(2);
		if (controllerVal < 1908) {
			// Auton 1
			lcdPrint(uart1, 2, "AUTON 1"); // High goal
			taskCreate(checkForward, TASK_DEFAULT_STACK_SIZE, NULL,
					TASK_PRIORITY_DEFAULT);
		} else {
			// Auton 2
			lcdPrint(uart1, 2, "AUTON 2"); // Low goal
			taskCreate(moveAuto, TASK_DEFAULT_STACK_SIZE, NULL,
					TASK_PRIORITY_DEFAULT);
		}
	}
}
