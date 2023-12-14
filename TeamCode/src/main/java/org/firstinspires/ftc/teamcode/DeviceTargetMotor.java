package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * DeviceTargetMotor Class
 * Created 11Nov2017 for the Greenwood FTC Robotics Club.
 * This class is designed for any DC motor and encoder combination.
 *
 * Objects created from this class will have the following status and behaviors:
 *    Status:
 *        + gearRatio = Mechanical advantage factor (value established upon object creation)
 *        + encoderCountsPerRev = Encoder counts per revolution (value established upon object creation)
 *        + radius = Radius of motorized implement (i.e., arm, wheel; value established upon object creation)
 *        + targetEngUnits = Final desired position of motorized assembly/implement (engineering units as
 *                established by the radius for a distance move or degrees for a rotational move)
 *        + targetCount = Final desired position of motorized assembly/implement (encoder counts)
 *    Behavior:
 *        + resetEncoder - Stop motor, reset encoder, and (re)set mode to "RUN_TO_POSITION"
 *        + stop - Set motor speed to zero and (re)set mode to "RUN_TO_POSITION"
 *        + setSpeed - Set motor speed (Note: This method is only needed if speed needs to be changed while
 *                a move operation is active)
 *        + getPosition - Return current position of motorized assembly in engineering units
 *        + getDegrees - Return current position of motorized assembly in degrees of rotation
 *        + goToRelativeDistance - Command relative distance move at given speed
 *        + goToAbsoluteDistance - Command absolute distance move at given speed
 *        + goToRelativeDegrees - Command relative angular move at given speed
 *        + goToAbsoluteDegrees - Command absolute angular move at given speed
 *        + isMoveDone - Monitor progress of a move initiated by a'goTo' method. Returns "true" if move is
 *                complete
 *        + goAtSpeed - Changes motor mode to "RUN_USING_ENCODER" (i.e., velocity) and commands given
 *                speed (-1.0 to 1.0). Must use the method stop() to reset "RUN_TO_POSITION" mode
 *    Interfaces:
 *        + robotMotor - DcMotor interface
 *
 * Revised 07January2019 */

class DeviceTargetMotor
{
    /* =======================================================
     * CLASS MEMBERS (i.e., Class Status)
     * ======================================================= */

    /* -------------------------------------------------------
     * Public (Shared) Class Members
     * ------------------------------------------------------- */
    DcMotor targetMotor;    // Motor to be monitored and controlled.

    double  gearRatio,            // gear ratio of motorized assembly
            encoderCountsPerRev,  // Encoder counts per axle revolution
            radius,               // Radius of implement (i.e., arm, wheel; engineering units)
            targetPosition = 0.0; // Final target position as calculated by a 'goTo' move method ( position in engineering units)

    int     targetCount = 0;      // Calculated final target position (encoder Counts)

    String  name;   // Motor name as configured on the Android device

    private boolean
            undefinedImplement = false, // No implement (e.g., arm, wheel) attached to or defined for the motor driven assembly
            undefinedEncoder = false,   // No encoder defined
            isDistanceMove = true,      // true = Distance move triggered; false = Degrees move triggered
            isBusyTrigger = false;      // Motor isBusy toggled off after last motion command


    /* =======================================================
     * CLASS CONSTRUCTOR
     * -------------------------------------------------------
     * Purpose: Establish and initialize a new TargetMotor (i.e., motor with encoder)
     * Input Parameters:
     *    + hwMap = Robot hardware
     *    + motorName = Motor name as configured on the Android device via the Robot Controller app
     *    + motorForward = Motor direction (false=FORWARD, true=REVERSE)
     *    + motorEncoderCountsPerRev = Encoder counts per axle revolution
     *    + motorGearRatio = gear ratio of motorized assembly
     *    + implementRadius = Radius of implement (i.e., arm, wheel; engineering units will be applied to all future 'goTo' and 'getPosition' methods)
     * Operations:
     *   + Link robotMotor to the actual installed motor given the motor name as configured on the Android phone.
     *   + Configure motor direction (FORWARD/REVERSE)
     *   + Reset motor encoder
     *   + Set motor mode to "RUN_TO_POSITION"
     *   + Initialize public variables targetCount and targetEngUnits
     *   + Set the initial position of the new servo object (0-1.0 = 0-180 degrees)
     *   + Capture motorized assembly attributes
     * Additional Notes:
     *   + Five constructors are available for object creation
     *   + The first constructor defines all the possible data needed to take full advantage of this class
     *   + If encoderCountsPerRev is not defined, all 'goTo...Distance' moves will be defined in encoder counts. 'goTo...Degrees' will be nonfunctional.
     *   + If radius is not defined, all 'goTo...Distance' moves will be defined in encoder counts. 'goTo...Degrees' will not be impacted.
     * ======================================================= */

    // Constructor #1
    DeviceTargetMotor(HardwareMap hwMap, String motorName, boolean motorDirection, double motorEncoderCountsPerRev, double motorGearRatio, double implementRadius) {

        targetMotor = hwMap.dcMotor.get(name = motorName);  // Link motor to installed hardware, and save name for later reference

        if (motorDirection)   // Configure motor direction
            targetMotor.setDirection(DcMotor.Direction.REVERSE);
        else
            targetMotor.setDirection(DcMotor.Direction.FORWARD);

        // Capture motor attributes for use in later calculations
        gearRatio = motorGearRatio;
        encoderCountsPerRev = motorEncoderCountsPerRev;
        radius = implementRadius;
    }

    // Constructor #2 (No Radius)
    DeviceTargetMotor(HardwareMap hwMap, String motorName, boolean motorForward, double motorEncoderCountsPerRev, double motorGearRatio) {
        this(hwMap, motorName,motorForward,motorEncoderCountsPerRev,motorGearRatio, 0.0);
        undefinedImplement = true;
    }

    // Constructor #3 (No Radius and Gear Ratio)
    DeviceTargetMotor(HardwareMap hwMap, String motorName, boolean motorForward, double motorEncoderCountsPerRev) {
        this(hwMap,motorName,motorForward,motorEncoderCountsPerRev, 1.0, 0.0);
        undefinedImplement = true;
    }

    // Constructor #4 (No Radius, Gear Ratio, and Encoder Counts Per Revolution)
    DeviceTargetMotor(HardwareMap hwMap, String motorName, boolean motorForward) {
        this(hwMap,motorName,motorForward,1.0, 1.0, 0.0);
        undefinedImplement = true;
        undefinedEncoder = true;
    }

    // Constructor #5 (No Radius, Gear Ratio, Encoder Counts Per Revolution, and Direction)
    DeviceTargetMotor(HardwareMap hwMap, String motorName) {
        this(hwMap,motorName,true,1.0,1.0,0.0);
        undefinedImplement = true;
        undefinedEncoder = true;
    }


    /* =======================================================
     * CLASS METHODS (i.e., Class Behavior)
     * ======================================================= */

    /* -------------------------------------------------------
     * Method: resetEncoder
     * Purpose: Initialize the motor. Stop motor, reset encoder, and set "RUN_TO_POSITION" mode
     * ------------------------------------------------------- */
    void resetEncoder() {
        targetMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Stop motor and reset encoder
        targetMotor.setTargetPosition(0);
        targetMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);         // Set mode to "RUN_TO_POSITION"
        targetCount = 0;        // Reset target status values
        targetPosition = 0.0;
    }


    /* -------------------------------------------------------
     * Method: stop
     * Purpose: Stop motor
     * Note: This method is useful after executing goAtSpeed(), but it provides no braking.
     * ------------------------------------------------------- */
    void stop () {
        targetMotor.setPower(0.0);   // Stop motor
    }


    /* -------------------------------------------------------
     * Method: setSpeed
     * Purpose: Modify speed while a move command is executing.
     * Input Parameters:
     *    + speed = Desired velocity (-1.0 to 1.0 = -100% to 100% configured max speed)
     * Note: For position/target moves, the absolute value of speed is used
     * ------------------------------------------------------- */
    void setSpeed(double speed) {
        targetMotor.setPower(speed);  // Set new motor speed
    }


    /* -------------------------------------------------------
     * Method: getPosition
     * Purpose: Returns current motorized assembly position in engineering units corresponding to the radius
     * ------------------------------------------------------- */
    double getPosition () {

        if (undefinedEncoder || undefinedImplement)
            // If encoder or radius is undefined, return encoder counts
            return (double)targetMotor.getCurrentPosition();
        else
            // If encoder and radius are undefined, calculate current position in engineering units
            return (((double)targetMotor.getCurrentPosition()) * 2.0 * radius * Math.PI) / (gearRatio * encoderCountsPerRev);
    }


    /* -------------------------------------------------------
     * Method: getDegrees
     * Purpose: Returns current motorized assembly position in engineering units corresponding to the radius
     * ------------------------------------------------------- */
    double getDegrees ()
    {
        if (undefinedEncoder)
            // If encoder is undefined, return zero
            return 0;
        else
            // If encoder is defined, return position in degrees
            return targetMotor.getCurrentPosition() * 360.0 / (gearRatio*encoderCountsPerRev);
    }


    /* -------------------------------------------------------
     * Method: goToRelativeDistance
     * Purpose: Calculate new relative target position and initiate move given new relative target distance and speed
     * Input Parameters:
     *    + relativeTargetDistance = Desired distance from current position
     *    + speed = velocity (0-1.0 = 0-100% maximum velocity)
     * Returns calculated target (encoder counts)
     * Additional Notes:
     *    + This method only initiates a move; the move may require numerous program scans for completion
     *    + Use the "isMoveDone" method to monitor move progress
     * ------------------------------------------------------- */
    int goToRelativeDistance(double relativeTargetDistance, double speed) {
        targetPosition = relativeTargetDistance + this.getPosition(); // Calculate new target distance in engineering units
        this.goToDistance(speed); // Call private method "goToDistance" in "this" class to calculate target (encoder counts) and to initiate the move
        return targetCount;       // Although targetCount is publicly available, it is also returned as a result of this method
    }


    /* -------------------------------------------------------
     * Method: goToAbsoluteDistance
     * Purpose: Calculate new absolute target position and initiate move given new absolute target distance and speed
     * Input Parameters:
     *    + absoluteTarget = Desired target with respect to home position (i.e., zero position)
     *    + speed = velocity (0-1.0 = 0-100% maximum velocity)
     * Returns calculated target (encoder counts)
     * Additional Notes:
     *    + This method only initiates a move; the move may require numerous program scans for completion
     *    + Use the "isMoveDone" method to monitor move progress
     * ------------------------------------------------------- */
    int goToAbsoluteDistance(double absoluteTarget, double speed) {
        targetPosition = absoluteTarget; // The new target is the given absolute target
        this.goToDistance(speed); // Call private method "goToDistance" in "this" class to calculate target (encoder counts) and to initiate the move
        return targetCount;       // Although targetCount is publicly available, it is also returned as a rsult of this method
    }


    /* -------------------------------------------------------
     * Method: goToRelativeDegrees
     * Purpose: Calculate new relative target position and initiate move given new relative target angle and speed
     * Input Parameters:
     *    + relativeTargetInDegrees = Desired angular movement (degrees) from current position (degrees)
     *    + speed = velocity (0-1.0 = 0-100% maximum velocity)
     * Returns calculated target (encoder counts)
     * Additional Notes:
     *    + This method only initiates a move; the move may require numerous program scans for completion
     *    + Use the "isMoveDone" method to monitor move progress
     * ------------------------------------------------------- */
    int goToRelativeDegrees(double relativeTargetInDegrees, double speed) {
        targetPosition = relativeTargetInDegrees + this.getDegrees();  // Calculate new target distance in degrees
        this.goToDegrees(speed);  // Call private method "goToDegrees" in "this" class to calculate target (encoder counts) and to initiate the move
        return targetCount;       // Although targetCount is publicly available, it is also returned as a rsult of this method
    }


    /* -------------------------------------------------------
     * Method: goToAbsoluteDegrees
     * Purpose: Calculate new absolute target position and initiate move given new absolute target angle and speed
     * Input Parameters:
     *    + absoluteTargetInDegrees = Desired angular movement (degrees) with respect to home position (sero position)
     *    + speed = velocity (0-1.0 = 0-100% maximum velocity)
     * Returns calculated target (encoder counts)
     * Additional Notes:
     *    + This method only initiates a move; the move may require numerous program scans for completion
     *    + Use the "isMoveDone" method to monitor move progress
     * ------------------------------------------------------- */
    int goToAbsoluteDegrees(double absoluteTargerInDegrees, double speed) {
        targetPosition = absoluteTargerInDegrees;  // The new target is the given absolute target
        this.goToDegrees(speed);  // Call private method "goToDegrees" in "this" class to calculate target (encoder counts) and to initiate the move
        return targetCount;       // Although targetCount is publicly available, it is also returned as a rsult of this method
    }


    /* -------------------------------------------------------
     * Method: isMoveDone
     * Purpose: Determine if 'goTo' position move is complete
     * Input Parameters:
     *    + targetDelta = The maximum allowed error from target position to actual position (engineering units for distance moves, degrees for angular moves)
     * Returns "true" if target achieved
     * ------------------------------------------------------- */
    boolean isMoveDone(double targetDelta) {
        int counts = targetMotor.getCurrentPosition(),  // Current motor position (encoder counts)
            countDelta;  // Allowable error from target (encoder counts)

        // Convert delta from engineering units to encoder counts
        if (isDistanceMove) { // If linear move ...
            if (undefinedImplement) // Delta must be provided in encoder counts in implement radius is not defined for a linear move
                countDelta = (int) targetDelta;
            else // Convert engineering units for a linear move to encoder counts.
                countDelta = (int)(targetDelta*gearRatio*encoderCountsPerRev/(2.0*radius*Math.PI));
        }
        else // if angular move, convert degrees to encoder counts
            countDelta = (int)(targetDelta*gearRatio*encoderCountsPerRev/360.0);

        /* If the motor is not busy after entering the target window, reset the isbusyTrigger to false.
         * Why needed? The targetMotor.isBusy() variable may toggle on and off after a move is complete. If the motor
         * is a component of a multi-motor system (e.g., drive train, multi-joint arm), the toggling targetMotor.isBusy()
          * variables for the motors in the system may prevent identification of system move complete. */
        if ((counts <= targetCount + countDelta) && (counts >= targetCount - countDelta) && !targetMotor.isBusy())
            isBusyTrigger = false;

         // If the motor is within the target window ( target-delta <= actual position <= target+delta) and the motor is not busy (i.e., actively moving), then the move is complete
        return (counts <= targetCount + countDelta) && (counts >= targetCount - countDelta) && !isBusyTrigger;
    }


    /* -------------------------------------------------------
     * Method: goAtSpeed
     * Purpose: Command the motor to run at a constant velocity given speed
     * Input Parameters:
     *    + speed = Desired velocity (-1.0 to 1.0 = -100% to 100% configured max speed)
     * Additional Notes:
     *    + To change speed, use the "setSpeed" method
     *    + To stop halt motion, use the "stop" method
     *    + The "stop" or "resetEncoders" method must be used before initiating a new 'goTo' move; otherwise, the motor will be in the wrong mode.
     * ------------------------------------------------------- */
    void goAtSpeed(double speed) {
        targetMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Change motor mode to "RUN_USING_ENCODER"
        targetMotor.setPower(speed);  // Set motor speed (-1.0 to 1.0 = -100% to 100% configured max speed)
    }


    /* -------------------------------------------------------
     * PRIVATE METHODS - not available for use outside of this class
     * -------------------------------------------------------*/

    /* -------------------------------------------------------
     * Method:goToDistance
     * Purpose: Calculate target (encoder counts) and initiate move
     * Input Parameters:
     *    + speed = Desired velocity (0-1.0 = 0-100% configured max speed)
     * ------------------------------------------------------- */
    private void goToDistance (double speed) {
        if (undefinedEncoder || undefinedImplement)
            // If encoder or radius are undefined, target count is the target position
            targetCount = (int)(targetPosition);
        else
            // Otherwise, use radius, gear ratio, and encoder counts to convert target position from engineering units to encoder counts
            targetCount = (int)(targetPosition*gearRatio*encoderCountsPerRev/(2.0*radius*Math.PI));

        targetMotor.setTargetPosition(targetCount);  // Initiate move given target (encoder counts)

        targetMotor.setPower(speed);  // Set motor speed

        isBusyTrigger = true;      // Move command triggered.
        isDistanceMove = true;    // Distance move triggered.
    }


    /* -------------------------------------------------------
     * Method:goToDegrees
     * Purpose: Calculate target (encoder counts) and initiate move
     * Input Parameters:
     *    + speed = Desired velocity (0-1.0 = 0-100% configured max speed)
     * ------------------------------------------------------- */
    private void goToDegrees(double speed)
    {
        if (undefinedEncoder)
            // If encoder is undefined, target count is set to zero
            targetCount = 0;
        else
            // Otherwise, use gear ratio and encoder counts to convert target position from engineering units to encoder counts
            targetCount = (int) (targetPosition * gearRatio * encoderCountsPerRev / 360.0);

        targetMotor.setTargetPosition(targetCount);  // Initiate move given target (encoder counts)

        targetMotor.setPower(speed);  // Set motor speed

        isBusyTrigger = true;      // Move command triggered.
        isDistanceMove = false;    // Degrees move triggered.
    }
}
