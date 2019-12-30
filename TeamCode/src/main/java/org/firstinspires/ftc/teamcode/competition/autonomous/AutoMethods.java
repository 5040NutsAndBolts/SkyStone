package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.competition.hardware.GrabbingMech;
import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.competition.helperclasses.HelperMethods;

public abstract class AutoMethods extends LinearOpMode {

    protected Hardware robot = new Hardware();
    protected MecanumDrive drive;
    /**
     * Moves the robot to a specific x-axis coordinate
     * @param endPosition X coordinate the robot will end at
     * @param thresholdPercent How close the robot will get to the target position
     */
    protected void xAxisMoveTo(double endPosition, double thresholdPercent) {
        /* TODO: Make it so the robot will move parallel to the x/y axis no matter rotation
            currently, it only works properly if the robot theta is 0
        */
        while (opModeIsActive() && !HelperMethods.inThreshold(robot.x, endPosition, thresholdPercent)) {
            updateOdometryTeleop();
            if (robot.x < endPosition)
                drive.drive(-.5, 0, 0);
            else if (robot.x > endPosition)
                drive.drive(.5, 0, 0);
        }
        drive.hardBrakeMotors();
    }

    /**
     * Moves the robot to a specific y-axis coordinate
     * @param endPosition Y coordinate the robot will end at
     * @param thresholdPercent How close the robot will get to the target position
     */
    protected void yAxisMoveTo(double endPosition, double thresholdPercent) {
        /* TODO: Make it so the robot will move parallel to the x/y axis no matter rotation
            currently, it only works properly if the robot theta is 0
        */
        while (opModeIsActive() && !HelperMethods.inThreshold(robot.y, endPosition, thresholdPercent)) {
            updateOdometryTeleop();
            if (robot.y < endPosition)
                drive.drive(0, .5, 0);
            else if (robot.y > endPosition)
                drive.drive(0, -.5, 0);
        }
        drive.hardBrakeMotors();
    }

    /**
     * Rotates the robot a specified number of degrees (relative to its current position)
     * @param degrees Degrees to rotate the robot
     * @param thresholdPercent How close the robot will get to the target position
     */
    protected void rotateDeg(double degrees, double thresholdPercent) {
        double endTheta = robot.theta + Math.toRadians(degrees);
        rotateToDeg(endTheta, thresholdPercent);
    }

    /**
     * Rotates the robot to a specified angle (in degrees)
     * @param endAngle Angle the robot will rotate to
     * @param thresholdPercent How close the robot will get to the target position
     */
    protected void rotateToDeg(double endAngle, double thresholdPercent) {
        endAngle = Math.toRadians(endAngle)%(2*Math.PI);
        if (robot.theta > endAngle) {
            while (opModeIsActive() && !HelperMethods.inThreshold(robot.theta, endAngle, thresholdPercent)) {
                updateOdometryTeleop();
                drive.drive(0, 0, .3);
            }
        }
        else {
            while (opModeIsActive() && !HelperMethods.inThreshold(robot.theta, endAngle, thresholdPercent)) {
                updateOdometryTeleop();
                drive.drive(0, 0, -.3);
            }
        }
        drive.hardBrakeMotors();
    }

    /**
     * Updates the robot position and displays it in the telemetry
     */
    protected void updateOdometryTeleop() {
        robot.updatePositionRoadRunner();
        telemetry.addData("X Position", robot.x);
        telemetry.addData("Y Position", robot.y);
        telemetry.addData("Rotation", robot.theta);
        telemetry.update();
    }

    /**
     * Does all the setting up for the gyro initialization and waits for opmode to start
     */
    protected void initGyroscope() {
        // Gyro setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters);
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while(opModeIsActive() && !robot.imu.isGyroCalibrated()) {
            telemetry.addData("imu calibration", robot.imu.isGyroCalibrated());
            telemetry.addLine();
            updateOdometryTeleop();
        }
    }

    /**
     * Brakes the motors for a specified amount of time
     * @param seconds Time in seconds to wait until the brakes release
     */
    protected void waitTime(double seconds) {
        long endTime = System.currentTimeMillis() + (int)(seconds*1000);
        while (System.currentTimeMillis() < endTime && opModeIsActive())
            drive.hardBrakeMotors();
    }

    protected double powerRampAngle(double startPoint, double endPoint) {
        if (startPoint > Math.PI)
            return .13 + .87 * Math.sin(Math.PI * ((robot.theta - startPoint) / (endPoint - startPoint)));
        else
            return .13 + .87 * Math.sin(-Math.PI * ((robot.theta - startPoint) / (endPoint - startPoint)));
    }

    /**
     * Displays position data for the end of autonomous
     */
    protected void displayEndAuto() {
        while(opModeIsActive()) {
            telemetry.addLine("Auto has ended");
            telemetry.addLine("==========");
            updateOdometryTeleop();
            drive.hardBrakeMotors();
        }
    }
}