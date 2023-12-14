package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * AssemblyDriveTrain Class
 * Created 19Nov2017 for the Greenwood FTC Robotics Club.
 * This class is designed for a balanced two motor drive train.
 *
 * Objects created from this class will have the following status and behaviors:
 *    Status:
 *        + leftSpeed = Commanded left motor speed (-1.0 to 1.0; Note: Position moves use the absolute value of the given speed
 *        + rightSpeed = Commanded right motor speed (-1.0 to 1.0)
 *    Behavior:
 *        + resetEncoders - Stop drive train, reset encoders, and set Run-to-Position mode
 *        + stop - Stop drive train and set Run-to-Position mode
 *        + setSpeed - Adjust drive train speed. (If drive train at target, the drive train will stay stopped)
 *        + goStraightToTarget - Set new drive train target and speed - straight line move
 *        + goStraightAtSpeed - Set 'velocity' mode, set speed, and go. (No target defined)
 *        + goRoundToTarget - Set new drive train target and speed - curve move. (Different targets for left and right drive motors)
 *        + goRoundAtSpeed - Set 'velocity' mode, set speed, and go. (Different speeds for left and right drive motors)
 *        + turnCwToTarget - Set new drive train target and speed - pivot clockwise on drive train axis
 *        + turnCcwToTarget - Set new drive train target and speed - pivot counterclockwise on drive train axis
 *        + isMoveDone - Check move status when driving to target
 *    Interfaces and Objects:
 *        + motorLeft - DeviceTargetMotor object
 *        + motorRight - DeviceTargetMotor object
 * Revised 01Dec2017 */


public class AssemblyDriveTrain
{
    /* =======================================================
     * CLASS MEMBERS (i.e., Class Status)
     * ======================================================= */

    /* -------------------------------------------------------
     * Public (Shared) Class Members
     * ------------------------------------------------------- */

    // Robot object definition
    public DeviceTargetMotor
            motorLeft,      // Left drive motor object
            motorRight;     // Right drive motor object

    public double
            leftSpeed = 0,  // Current/commanded left drive motor speed (-1.0 to 1.0 = -100% to 100% configured max speed)
            rightSpeed = 0; // Current/commanded right drive motor speed (-1.0 to 1.0 = -100% to 100% configured max speed)


    /* -------------------------------------------------------
     * Private (Concealed) Class Members
     * -- none --
     * ------------------------------------------------------- */



    /* =======================================================
     * CLASS CONSTRUCTOR
     * -------------------------------------------------------
     * Purpose: Establish DeviceTrainMotor objects for the left and right drive motors
     * Input Parameters:
     *    + hwMap = Robot hardware
     *    + leftMotorName = Left drive motor name as configured on the Android device via the Robot Controller app
     *    + leftMotorDirection = Left motor direction (false=FORWARD, true=REVERSE)
     *    + rightMotorName = Right drive motor name as configured on the Android device via the Robot Controller app
     *    + rightMotorDirection = Right motor direction (false=FORWARD, true=REVERSE)
     *    + motorEncoderCountsPerRev = Encoder counts per axle revolution
     *    + motorGearRatio = gear ratio of motorized assembly
     *    + implementRadius = Radius of implement (i.e., arm, wheel; engineering units will be applied to all future 'goTo' and 'getPosition' methods)
     * Operations:
     *   + Create DeviceTArgetMotor objects for the left and right drive train motors
     * ======================================================= */

    AssemblyDriveTrain(HardwareMap hwMap, String leftMotorName, boolean leftMotorDirection, String rightMotorName, boolean rightMotorDirection, double motorEncoderCountsPerRev, double motorGearRatio, double wheelRadius) {
        motorLeft = new DeviceTargetMotor(hwMap,leftMotorName,leftMotorDirection,motorEncoderCountsPerRev,motorGearRatio,wheelRadius);
        motorRight = new DeviceTargetMotor(hwMap,rightMotorName,rightMotorDirection,motorEncoderCountsPerRev,motorGearRatio,wheelRadius);
    }



    /* =======================================================
     * CLASS METHODS (i.e., Class Behavior)
     * ======================================================= */

    /* -------------------------------------------------------
     * Method: resetEncoders
     * Purpose: Stop drive motors, reset encoders, (re)set "RUN_TO_POSITION" mode, and update drive speed status
     * Notes: This method employs the methods inherent in the DeviceMotorTarget class
     * ------------------------------------------------------- */
    public void resetEncoders () {
        motorLeft.resetEncoder();
        motorRight.resetEncoder();
        leftSpeed = 0;
        rightSpeed = 0;
    }


    /* -------------------------------------------------------
     * Method: stop
     * Purpose: Stop drive motors and update drive speed status
     * Additional Notes: This method employs the methods inherent in the DeviceMotorTarget class
     * ------------------------------------------------------- */
    public void stop () {
        motorLeft.stop();
        motorRight.stop();
        leftSpeed = 0;
        rightSpeed = 0;
    }


    /* -------------------------------------------------------
     * Method: setSpeed
     * Purpose: Modify drive speed while a move command is executing.
     * Input Parameters:
     *    + speed = Desired drive train velocity (-1.0 to 1.0 = -100% to 100% configured max speed)
     * Additional Note:
     *    + This method will adjust speed to maintain the trajectory of a round/curve move. Input speed is the speed of the fastest wheel
     *    + This method employs the methods inherent in the DeviceMotorTarget class
     * ------------------------------------------------------- */
    public void setSpeed (double speed) {

        if (leftSpeed > rightSpeed) {
            // If the left motor is running faster, calculate the speed of the right motor based on the original right-to-left speed ratio; ...
            rightSpeed = speed * rightSpeed / leftSpeed;
            leftSpeed = speed;
        }
        else {
            // ... otherwise, calculate the speed of the left motor based on the original left-to-right speed ratio
            leftSpeed = speed * leftSpeed / rightSpeed;
            rightSpeed = speed;
        }

        motorLeft.setSpeed(leftSpeed);      // Set new left motor speed
        motorRight.setSpeed(rightSpeed);    // Set new right motor speed
    }


    /* -------------------------------------------------------
     * Method: goStraightToTarget
     * Purpose: Move the drive train in a straight line from its current position to the given distance (i.e., relative move)
     * Input Parameters:
     *    + distance = Desired relative distance from current position in the same engineering units as as the wheel radius
     *    + speed = Desired drive train velocity (-1.0 to 1.0 = -100% to 100% configured max speed)
     * Note: This method employs the methods inherent in the DeviceMotorTarget class
     * ------------------------------------------------------- */
    public void goStraightToTarget(double distance, double speed) {
        leftSpeed = speed;
        rightSpeed = speed;
        motorLeft.goToRelativeDistance(distance,leftSpeed);
        motorRight.goToRelativeDistance(distance,rightSpeed);
    }


    /* -------------------------------------------------------
     * Method: goStraightAtSpeed
     * Purpose: Move the drive train in a straight line at constant speed
     * Input Parameters:
     *    + speed = Desired drive train velocity (-1.0 to 1.0 = -100% to 100% configured max speed)
     * Note:
     *    + To return the drive train to target operation (i.e., RUN_TO_POSITION), execute the method "stop" in this class.
     *    + This method employs the methods inherent in the DeviceMotorTarget class
     * ------------------------------------------------------- */
    public void goStraightAtSpeed(double speed) {
        leftSpeed = speed;
        rightSpeed = speed;
        motorLeft.goAtSpeed(leftSpeed);
        motorRight.goAtSpeed(rightSpeed);
    }


    /* -------------------------------------------------------
     * Method: goRoundToTarget
     * Purpose: Move the drive train in a curve from its current position to the given distance of each drive wheel (i.e., relative moves)
     * Input Parameters:
     *    + leftDistance = Desired relative distance from the current left wheel position in the same engineering units as as the wheel radius
     *    + rightDistance = Desired relative distance from the current right wheel position in the same engineering units as as the wheel radius
     *    + speed = Desired velocity of the drive wheel with the furthest distance to travel (-1.0 to 1.0 = -100% to 100% configured max speed)
     * Note:
     *    + This method will calculate the slower speed of the drive wheel that is commanded to travel the shorter distance
     *    + This method employs the methods inherent in the DeviceMotorTarget class
     * ------------------------------------------------------- */
    public void goRoundToTarget(double leftDistance, double rightDistance, double speed) {

        if (Math.abs(leftDistance) > Math.abs(rightDistance)) {
            // If the left drive wheel must travel the furthest, calculate the speed of the right drive wheel based on the right-to-left distance ratio; ...
            rightSpeed = speed * rightDistance / leftDistance;
            leftSpeed = speed;
        }
        else {
            // ... otherwise, calculate the speed of the left drive wheel based on the left-to-right distance ratio; ...
            leftSpeed = speed * leftDistance / rightDistance;
            rightSpeed = speed;
        }

        motorLeft.goToRelativeDistance(leftDistance,leftSpeed);
        motorRight.goToRelativeDistance(rightDistance,rightSpeed);
    }


    /* -------------------------------------------------------
     * Method: goRoundAtSpeed
     * Purpose: Move the drive train in a curve at constant speed
     * Input Parameters:
     *    + leftSpeed = Desired left drive wheel velocity (-1.0 to 1.0 = -100% to 100% configured max speed)
     *    + rightSpeed = Desired right drive wheel velocity (-1.0 to 1.0 = -100% to 100% configured max speed)
     * Note:
     *    + To return the drive train to target operation (i.e., RUN_TO_POSITION), execute the method "stop" in this class.
     *    + This method employs the methods inherent in the DeviceMotorTarget class
     * ------------------------------------------------------- */
    public void goRoundAtSpeed(double leftSpeed, double rightSpeed) {
        motorLeft.goAtSpeed(leftSpeed);
        motorRight.goAtSpeed(rightSpeed);
    }


    /* -------------------------------------------------------
     * Method: turnCwToTarget
     * Purpose: Rotate/pivot the relicRobot clockwise
     * Input Parameters:
     *    + distance = Desired relative distance the left wheel and the (reversed) right wheel must travel in the same engineering units as as the wheel radius
     *    + speed = Desired velocity of the drive wheels (-1.0 to 1.0 = -100% to 100% configured max speed)
     * Note:
     *    + This method employs the methods inherent in the DeviceMotorTarget class
     * ------------------------------------------------------- */
    public void turnCwToTarget (double distance, double speed) {
        leftSpeed = speed;
        rightSpeed = speed;
        motorLeft.goToRelativeDistance(distance,leftSpeed);
        motorRight.goToRelativeDistance(-distance,rightSpeed);
    }


    /* -------------------------------------------------------
     * Method: turnCcwToTarget
     * Purpose: Rotate/pivot the relicRobot counterclockwise
     * Input Parameters:
     *    + distance = Desired relative distance the right wheel and the (reversed) left wheel must travel in the same engineering units as as the wheel radius
     *    + speed = Desired velocity of the drive wheels (-1.0 to 1.0 = -100% to 100% configured max speed)
     * Note:
     *    + This method employs the methods inherent in the DeviceMotorTarget class
     * ------------------------------------------------------- */
    public void turnCcwToTarget (double distance, double speed) {
        leftSpeed = speed;
        rightSpeed = speed;
        motorLeft.goToRelativeDistance(-distance,leftSpeed);
        motorRight.goToRelativeDistance(distance,rightSpeed);
    }


    /* -------------------------------------------------------
     * Method: isMoveDone
     * Purpose: Check the progress of a commanded target move
     * Return: true = left and right drive train wheel are at their target destination
     * Note:
     *    + This method employs the methods inherent in the DeviceMotorTarget class
     * ------------------------------------------------------- */
    public boolean isMoveDone (double targetDelta) {
        return motorLeft.isMoveDone(targetDelta) && motorRight.isMoveDone(targetDelta);
    }


}  // End AssemblyDriveTrain Class
