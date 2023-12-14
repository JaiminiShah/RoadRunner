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


public class mecanumDrive1 extends FourWheelDrive
{
    /* =======================================================
     * CLASS MEMBERS (i.e., Class Status)
     * ======================================================= */

    /* -------------------------------------------------------
     * Public (Shared) Class Members
     * ------------------------------------------------------- */

    // Robot object definition




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
     *    + rightMotorName = Rive might drotor name as configured on the Android device via the Robot Controller app
     *    + rightMotorDirection = Right motor direction (false=FORWARD, true=REVERSE)
     *    + motorEncoderCountsPerRev = Encoder counts per axle revolution
     *    + motorGearRatio = gear ratio of motorized assembly
     *    + implementRadius = Radius of implement (i.e., arm, wheel; engineering units will be applied to all future 'goTo' and 'getPosition' methods)
     * Operations:
     *   + Create DeviceTArgetMotor objects for the left and right drive train motors
     * ======================================================= */

    public mecanumDrive1(HardwareMap hwMap, String frontLeftMotorName, boolean frontLeftMotorDirection,
                         String frontRightMotorName, boolean frontRightMotorDirection,
                         String rearLeftMotorName, boolean rearLeftMotorDirection,
                         String rearRightMotorName, boolean rearRightMotorDirection,
                         double motorEncoderCountsPerRev, double motorGearRatio, double wheelRadius) {


        super(hwMap, frontLeftMotorName, frontLeftMotorDirection,
                frontRightMotorName, frontRightMotorDirection, rearLeftMotorName,
                rearLeftMotorDirection, rearRightMotorName, rearRightMotorDirection,
                motorEncoderCountsPerRev, motorGearRatio, wheelRadius);

    }



    /* =======================================================
     * CLASS METHODS (i.e., Class Behavior)
     * ======================================================= */

    /* -------------------------------------------------------
     * Method: resetEncoders
     * Purpose: Stop drive motors, reset encoders, (re)set "RUN_TO_POSITION" mode, and update drive speed status
     * Notes: This method employs the methods inherent in the DeviceMotorTarget class
     * ------------------------------------------------------- */

    public void StrafeToTarget(double frontLeftDistance, double frontRightDistance,double speed) {

        front.goRoundToTarget(frontLeftDistance,frontRightDistance,speed);
        rear.goRoundToTarget(frontRightDistance,frontLeftDistance,speed);

}




    public void StrafeLeftToTarget(double distance, double speed) {

        this.StrafeToTarget(-distance, distance, speed);

    }



    public void StrafeRightToTarget(double distance, double speed) {

        this.StrafeToTarget(distance, -distance, speed);

    }
    public void RotateLeft(double distance, double speed){
        this.turnCwToTarget(distance,speed);
        //this.StrafeToTarget(0,-distance,speed);
    }
    public void RotateRight(double distance,double speed){
        this.turnCcwToTarget(distance,speed);
    }


}  // End AssemblyDriveTrain Class
