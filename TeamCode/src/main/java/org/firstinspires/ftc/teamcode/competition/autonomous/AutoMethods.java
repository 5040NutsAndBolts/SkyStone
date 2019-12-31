package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.PurePursuit.CheckPoint;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.PurePursuit.WayPoint;
import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.competition.helperclasses.ThreadPool.pool;

public abstract class AutoMethods extends LinearOpMode {

    protected Hardware robot = new Hardware();
    protected MecanumDrive drive;
    protected PurePursuit purePursuit;

    private boolean onRed = false;
    private boolean parkAgainstBridge = false;

    /**
     * Updates the robot position and displays it in the telemetry
     */
    protected void updateOdometryTelemetry() {
        robot.updatePositionRoadRunner();
        telemetry.addData("X Position", robot.x);
        telemetry.addData("Y Position", robot.y);
        telemetry.addData("Rotation", robot.theta);
        telemetry.update();
    }

    /**
     * Initializes all the hardware and pure pursuit for auto
     * Also has instructions for alliance and parking selection
     */
    protected void initAuto() {
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);
        drive.softBrakeMotors();
        purePursuit = new PurePursuit(robot);


        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x || gamepad2.x)
                onRed = false;
            if (gamepad1.b || gamepad2.b)
                onRed = true;

            if (gamepad1.y || gamepad2.y)
                parkAgainstBridge = true;
            if (gamepad1.a || gamepad2.a)
                parkAgainstBridge = false;

            telemetry.addLine("Use either controller for init process");
            telemetry.addLine();
            telemetry.addLine("Press X for blue and B for red alliances");
            telemetry.addLine("Press Y for parking against bridge A for against wall");
            telemetry.addLine();
            if (onRed)
                telemetry.addLine("RED alliance");
            else
                telemetry.addLine("BLUE alliance");
            if (parkAgainstBridge)
                telemetry.addLine("Park against neutral bridge");
            else
                telemetry.addLine("Park against wall");
            telemetry.addLine("==========");
            updateOdometryTelemetry();
        }
    }

    /**
     * Does all the setting up for the gyro initialization and waits for opmode to start
     */
    protected void initGyroscope() {
        robot.init(hardwareMap);
        // Gyro setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters);
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive() && !robot.imu.isGyroCalibrated()) {
            telemetry.addData("imu calibration", robot.imu.isGyroCalibrated());
            telemetry.addLine();
            initAuto();
        }
    }

    /**
     * Brakes the motors for a specified amount of time
     *
     * @param seconds Time in seconds to wait until the brakes release
     */
    protected void waitTime(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() < seconds && opModeIsActive())
            drive.hardBrakeMotors();
    }

    /**
     * Displays position data for the end of autonomous
     */
    protected void displayEndAuto() {
        while (opModeIsActive()) {
            telemetry.addLine("Auto has ended");
            telemetry.addLine("==========");
            updateOdometryTelemetry();
            drive.hardBrakeMotors();
        }
    }


    // ==============================
    // ==============================
    //
    //       PURE PURSUIT PATHS
    //
    // ==============================
    // ==============================

    /**
     * Flips the coordinates for a Pure Pursuit path from being relative from one side of the field to the other
     * For example, an auto written for blue can be flipped to run on red, or vice versa
     *
     * @param checkPoint Checkpoint of Pure Pursuit path
     * @param wayPoints  List of WayPoints of Pure Pursuit path
     */
    private void flipPurePursuitPath(CheckPoint checkPoint, ArrayList<WayPoint> wayPoints) {
        checkPoint.y *= -1;
        for (int i = 0; i < wayPoints.size(); i++)
            wayPoints.get(i).y *= -1;
    }

    /**
     * Runs a Pure Pursuit path with given data until it reaches a specific checkpoint
     *
     * @param checkPoint Checkpoint at which to stop running path
     * @param path       Pure Pursuit path to run the robot along
     * @param pathData   P.I.D. (Optional), look ahead distance, speed, and turn speed information for Pure Pursuit
     */
    public void runPurePursuitPath(CheckPoint checkPoint, ArrayList<WayPoint> path, double[] pathData) {
        // Flips the path to work for red alliance
        if (onRed)
            flipPurePursuitPath(checkPoint, path);

        // Initializes the pure pursuit path and declares the followPath() arguments
        double lookAheadDistance, speed, turnSpeed;
        if (pathData.length == 3) {
            purePursuit.initPath(path);
            lookAheadDistance = pathData[0];
            speed = pathData[1];
            turnSpeed = pathData[2];
        }
        else {
            purePursuit.initPath(path, pathData[0], pathData[1], pathData[2]);
            lookAheadDistance = pathData[3];
            speed = pathData[4];
            turnSpeed = pathData[5];
        }

        // Submits the checkpoint to the thread pool
        pool.submit(checkPoint);
        // Continuously runs the pure pursuit path until the robot hits the checkpoint
        while (opModeIsActive() && !checkPoint.isHit) {
            telemetry.addData("PID", purePursuit.pos.getPID());
            updateOdometryTelemetry();

            purePursuit.followPath(path, lookAheadDistance, speed, turnSpeed);
        }
        // Stop the robot and expel the checkpoint form the thread pool
        drive.hardBrakeMotors();
        checkPoint.terminate();

        // Flips the path back to its original
        if (onRed)
            flipPurePursuitPath(checkPoint, path);
    }

    // ===============
    // FOUNDATION AUTO
    // ===============

    public CheckPoint
            cp_foundationGrab = new CheckPoint(31.5, 12, 2, robot),
            cp_foundationPull = new CheckPoint(0, 4, 4, robot),
            cp_foundationPush = new CheckPoint(20, 36, 1, robot),
            cp_foundationPark = new CheckPoint(-1, 70, 2, robot);

    public ArrayList<WayPoint>
            wp_foundationGrab = new ArrayList<>(
            Arrays.asList(
                    new WayPoint(31.5, 12, 0)
            )),
            wp_foundationPull = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(0, 12, 0),
                            new WayPoint(0, 4, 0)
                    )),
            wp_foundationPush = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(-1, 48, 0),
                            new WayPoint(20, 36, 0)
                    )),
            wp_foundationPark = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(1, 45, 0),
                            new WayPoint(-1, 70, 0)
                    ));

    // ==========
    // SKYSTONE AUTO
    // ==========

}