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
    protected MecanumDrive drive = new MecanumDrive(robot);
    protected GrabbingMech grabbingMech = new GrabbingMech(robot);

    /**
     * Moves the robot to a specific x-axis coordinate
     * @param endPosition X coordinate the robot will end at
     * @param thresholdPercent How close the robot will get to the target position
     */
    protected void xAxisMoveTo(double endPosition, double thresholdPercent) {
        /* TODO: Make it so the robot will move parallel to the x/y axis no matter rotation
            currently, it only works properly if the robot theta is 0
        */
        while (opModeIsActive() && !HelperMethods.inThreshhold(robot.x, endPosition, thresholdPercent)) {
            updateOdometryTeleop();
            if (robot.theta > Math.PI) {
                if (robot.x < endPosition)
                    drive.drive(.5, 0, 0);
                else if (robot.x > endPosition)
                    drive.drive(-.5, 0, 0);
            }
            else {
                if (robot.x < endPosition)
                    drive.drive(-.5, 0, 0);
                else if (robot.x > endPosition)
                    drive.drive(.5, 0, 0);
            }
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
        while (opModeIsActive() && !HelperMethods.inThreshhold(robot.y, endPosition, thresholdPercent)) {
            updateOdometryTeleop();
            if (robot.y < endPosition)
                drive.drive(0,-.5,0);
            else if (robot.y > endPosition)
                drive.drive(0,.5,0);
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
        while (opModeIsActive() && !HelperMethods.inThreshhold(robot.theta, endTheta, thresholdPercent)) {
            updateOdometryTeleop();
            if (robot.theta > endTheta)
                drive.drive(0,0,.25);
            else if (robot.theta < endTheta)
                drive.drive(0,0,-.25);
        }
        drive.hardBrakeMotors();
    }

    /**
     * Rotates the robot to a specified angle (in degrees)
     * @param angle Angle the robot will rotate to
     * @param thresholdPercent How close the robot will get to the target position
     */
    protected void rotateToDeg(double angle, double thresholdPercent) {
        angle = Math.toRadians(angle);
        while (opModeIsActive() && !HelperMethods.inThreshhold(robot.theta, angle, thresholdPercent)) {
            updateOdometryTeleop();
            if (robot.theta > angle)
                drive.drive(0,0,.25);
            else if (robot.theta < angle)
                drive.drive(0,0,-.25);
        }
        drive.hardBrakeMotors();
    }

    protected void updateOdometryTeleop() {
        robot.updatePositionRoadRunner();
        telemetry.addData("X Position", robot.x);
        telemetry.addData("Y Position", robot.y);
        telemetry.addData("Rotation", robot.theta);
        telemetry.update();
    }

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
            telemetry.update();
        }
    }
}