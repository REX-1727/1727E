/** @file main.h
 * @brief Header file for global functions
 *
 * Any experienced C or C++ programmer knows the importance of header files. For those who
 * do not, a header file allows multiple files to reference functions in other files without
 * necessarily having to see the code (and therefore causing a multiple definition). To make
 * a function in "opcontrol.c", "auto.c", "main.c", or any other C file visible to the core
 * implementation files, prototype it here.
 *
 * This file is included by default in the predefined stubs in each VEX Cortex PROS Project.
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

#ifndef MAIN_H_

// This prevents multiple inclusion, which isn't bad for this file but is good practice
#define MAIN_H_

// Motors
#define FRONT_RIGHT 1
#define FRONT_LEFT 2
#define TOWER 3
#define CAT_1 4
#define CAT_2 5
#define CAT_3 6
#define CAT_4 7
#define INTAKES 8
#define BACK_LEFT 9
#define BACK_RIGHT 10

#define MAX_LAUNCH_VAL 950
#define AUTON_DECISION_VAL 1908
#define MULTIPLIER 0.63
#define POT_MAX_DIFF 600
#define NUM_AUTO_LAUNCHES 3
#define MOTOR_RESISTANCE -25
#define CAT_PARTIAL_POWER -63

#include <API.h>

// Allow usage of this file in C++ programs
#ifdef __cplusplus
extern "C" {
#endif

// A function prototype looks exactly like its declaration, but with a semicolon instead of
// actual code. If a function does not match a prototype, compile errors will occur.

// Prototypes for initialization, operator control and autonomous

/**
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
void autonomous();
/**
 * Runs pre-initialization code. This function will be started in kernel mode one time while the
 * VEX Cortex is starting up. As the scheduler is still paused, most API functions will fail.
 *
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO();
/**
 * Runs user initialization code. This function will be started in its own task with the default
 * priority and stack size once when the robot is starting up. It is possible that the VEXnet
 * communication link may not be fully established at this time, so reading from the VEX
 * Joystick may fail.
 *
 * This function should initialize most sensors (gyro, encoders, ultrasonics), LCDs, global
 * variables, and IMEs.
 *
 * This function must exit relatively promptly, or the operatorControl() and autonomous() tasks
 * will not start. An autonomous mode selection menu like the pre_auton() in other environments
 * can be implemented in this task if desired.
 */
void initialize();
/**
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl();
/**
* Joystick control of drive for opcontrol
*/
void checkDrive();
/**
 * Sets up tower and intakes to run at the same rate
 */
void setTowerAndIntake();
/**
 * Sets up catapult motors & control
 */
void setCatapultMotors();
/*
 * Sets catapult motors to full power
 * @param direction Bool to indicate which direction: true for positive and false for negative
 */
void setCatapultMotorsToFull(bool direction);
/*
 * Sets given port to full power in either direction
 * @param port Which motor port to set to full power
 * @param direction Bool to indicate which direction: true for positive and false for negative
 */
 /*
 * Set manual value for catapult motors
 */
 void runCatapultMotors(int power);
void setFullPower(int port, bool direction);
/*
 * Calculate battery percentage and display as bars on LCD
 */
void showBatteryOnLcd(int *lcd);
/*
 * Turns backlight on if power source is computer
 * Battery level is 0, so main battery is turned off and must be powered by computer if on
 * @return Bool indicating whether the backlight should be off or on
 */
 bool checkBattery(int *lcd);
/*
 * Grabs potentiometer value at a given moment (should be placed in loop) and prints to LCD
 */
void showPotVals(int *lcd, int port);
/*
 * Check if catapult should launch
 * @param ignore Value to ignore required by tasks
 */
void checkCatapult(void *ignore);
/*
 * Launches catapult after being pulled back
 * @param length Length of time to run motors
 */
void launchCatapult(int length);
/*
 * Holds catapult back in preparation for launch
 * @param targetDiff Target difference of pot
 */
void pullCatapultBack(int targetDiff);
/*
 * Initializes LCD values for battery checks & button toggles
 */bool *initLcdVals();
/*
 * Plays a sound
 */
void playSound();
/*
 * Stops all of the catapult motors
 */
void stopCatapultMotors();
/*
 * Checks whether LCD button 1 is pressed down and toggles backlights accordingly
 * @param lcd LCD to check
 * @param backlight Whether backlight is currently on
 */bool checkBacklight(int *lcd, bool backlight);
/*
 * Allows buttons to be used to control drive motors at full power for drive testing
 * Uses MULTIPLIER speed correction
 */
void checkForManualDrive();
/*
 * Autonomous option 1
 */
void checkForward();
/*
 * Autonomous option 2
 */
void moveAuto();
/*
 * Corrects wheel movement w/ IMEs to move straight
 */
void driveStraight();
/*
*  Variable delays
*/
void delayLaunch(int time);
/*
 * Task to handle catapult auto-launching
 */
extern TaskHandle catTask;
/*
 * Global flag to see whether catapult is being pulled back
 */
extern bool running;
/*
*  Global gyro object
*/
extern Gyro gy;
// End C++ export structure
#ifdef __cplusplus
}
#endif

#endif
