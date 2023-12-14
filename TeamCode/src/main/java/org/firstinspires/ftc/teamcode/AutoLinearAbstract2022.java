package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutoLinearAbstract2022 extends LinearOpMode {

    //This is a watered-down version of Green Machine's autonomous abstract from last year(2021)
    //I have done my best to simplify the code and make it more understandable
    //All code from last year is in a separate folder labeled "Examples From Last Year"

    /* =======================================================
     * CLASS MEMBERS (i.e., Class Status)
     * Common autonomous opmode members
     * ======================================================= */

    /* -------------------------------------------------------
     * Public (Shared) Class Members
     * Automated objects, timers, variables, constants
     * ------------------------------------------------------- */

    // OBJECTS
    mecanumDrive1
            driveTrain;

   // Where you would declare any additional motors
    // DeviceTargetMotor

    //Where you would declare any servos for the robot
    //DeviceTargetServo

    //declares timers that we use
    ElapsedTime
            generalTimer = new ElapsedTime(), // General/multipurpose timer
            autoTimer = new ElapsedTime();    // Autonomous timer



//  -------------------------Variables----------------------------------------------
    //any words we might need to store
    //String

    //True-false variables
    boolean
            safeStop;
    // any whole number variables
    //int

    // any decimal values
    //double


    // constants: declared here and can never be changed further in the program
    // however, changing one of these numbers will change it for all(speeds)
    final double
            MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES = .25,
            DRIVE_TRAIN_STRAIGHT_SPEED = 0.9,
            DRIVE_TRAIN_DEFAULT_SPEED = 0.7;

    final boolean
            FORWARD = false,
            REVERSE = true;

    /* =======================================================
     * CLASS METHODS (i.e., Class Behavior)
     * ======================================================= */

    /* -------------------------------------------------------
     * Method: runOpMode (Overridden Linear OpMode)
     * Purpose: Establish and initialize automated objects, and signal initialization completion
     * ------------------------------------------------------- */
    @Override
    public void runOpMode() {

        safeStop = false; //Used for stopping robot
        /* INITIALIZE ROBOT - ESTABLISH ROBOT OBJECTS */

        /* Drive Train constructor: hardwareMap, left motor name, left motor direction, right motor name, right motor direction,
                                    encoder counts per output shaft revolution, gear ratio, wheel radius */
        driveTrain = new mecanumDrive1(hardwareMap, "front_left_drive",FORWARD, "front_right_drive", REVERSE, "rear_left_drive", FORWARD, "rear_right_drive", REVERSE, 960, 0.66, 1.968504);

        /* Target-Motor constructor: hardwareMap, motor name, motor direction,
                              encoder counts per output shaft revolution, gear ratio, wheel radius */

        // Target-Servo constructor: hardwareMap, servo name, initial servo position

        // Notify drive station that robot objects are being initialized
        telemetry.addLine("Wait - Initializing Robot Objects");
        telemetry.update();

         /* Reset encoders and place motors into the 'Run-to-Position' mode
            Note: The initialization calls in the following methods could not be performed in the respective
           object constructors */
        driveTrain.resetEncoders();

        /* Lock drive train at current position */
        driveTrain.front.motorLeft.goToAbsoluteDistance(driveTrain.front.motorLeft.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.front.motorRight.goToAbsoluteDistance(driveTrain.front.motorRight.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.rear.motorLeft.goToAbsoluteDistance(driveTrain.rear.motorLeft.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.rear.motorRight.goToAbsoluteDistance(driveTrain.rear.motorRight.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);



        /* INITIALIZE ROBOT - SIGNAL INITIALIZATION COMPLETE */
        // Report initialization complete
        telemetry.addLine("Initialization Complete");
        telemetry.addLine("Hold for Start");
        telemetry.update();

        // WAIT FOR THE GAME TO START (driver presses PLAY)
        waitForStart();

        autoTimer.reset();  // Reset/restart the autotimer

        // GAME STARTED - BEGIN AUTONOMOUS OPERATIONS

    }






    /* -------------------------------------------------------
     * Method: driveTrainTelemetry
     * Purpose: Report the position and speed of the drive train wheels
     * ------------------------------------------------------- */

    void DriveTrainTelemetry() {

        telemetry.addLine("Left Drive Front Motor");
        telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.front.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.front.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts", driveTrain.front.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts", driveTrain.front.motorLeft.targetCount);
        telemetry.addData("  Is Busy", driveTrain.front.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed", driveTrain.front.leftSpeed);


        telemetry.addLine("Left Drive Rear Motor");
        telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.rear.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.rear.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts", driveTrain.rear.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts", driveTrain.rear.motorLeft.targetCount);
        telemetry.addData("  Is Busy", driveTrain.rear.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed", driveTrain.rear.leftSpeed);

        telemetry.addLine("Right Drive Front Motor");
        telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.front.motorRight.getPosition());
        telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.front.motorRight.targetPosition);
        telemetry.addData("  Position in Counts", driveTrain.front.motorRight.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts", driveTrain.front.motorRight.targetCount);
        telemetry.addData("  Is Busy", driveTrain.front.motorRight.targetMotor.isBusy());
        telemetry.addData("  Speed", driveTrain.front.rightSpeed);

        telemetry.addLine("Right Drive Rear Motor");
        telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.rear.motorRight.getPosition());
        telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.rear.motorRight.targetPosition);
        telemetry.addData("  Position in Counts", driveTrain.rear.motorRight.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts", driveTrain.rear.motorRight.targetCount);
        telemetry.addData("  Is Busy", driveTrain.rear.motorRight.targetMotor.isBusy());
        telemetry.addData("  Speed", driveTrain.rear.rightSpeed);
    }

    void motorTelemetryDegrees (DeviceTargetMotor motor) {
        telemetry.addLine();
        telemetry.addLine(motor.name);
        telemetry.addData(" Position in Degrees", "%.2f degrees ", motor.getDegrees());
        telemetry.addData(" Position in Counts", motor.targetMotor.getCurrentPosition());
    }

    void motorTelemetry (DeviceTargetMotor motor) {
        telemetry.addLine();
        telemetry.addLine(motor.name);
        telemetry.addData(" Position in EU", "%.2f EU ", motor.getPosition());
        telemetry.addData(" Position in Counts", motor.targetMotor.getCurrentPosition());
    }


    boolean Kill ( double autoTime) {
        boolean eStop;

        if (!opModeIsActive() || autoTimer.seconds() >= autoTime || safeStop) {
            driveTrain.stop();
            //Stops movement of shooter

            eStop = true;
        }
        else
            eStop = false;
        return eStop;
    }
}