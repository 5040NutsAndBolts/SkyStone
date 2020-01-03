package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.PurePursuit.*;
import org.firstinspires.ftc.teamcode.competition.autonomous.vision.SkystonePipeline;
import org.firstinspires.ftc.teamcode.competition.hardware.*;
import org.firstinspires.ftc.teamcode.competition.helperclasses.ThreadPool;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.competition.autonomous.vision.SkystonePipeline.screenPosition;
import static org.firstinspires.ftc.teamcode.competition.helperclasses.HelperMethods.inThreshold;
import static org.firstinspires.ftc.teamcode.competition.helperclasses.ThreadPool.pool;

public abstract class AutoMethods extends LinearOpMode {

    protected Hardware robot = new Hardware();
    protected MecanumDrive drive;
    protected IntakeMech intake;
    protected FoundationGrabbers foundationGrabbers;
    protected LiftMech lift;
    protected PurePursuit purePursuit;
    protected OpenCvCamera phoneCamera;
    protected ElapsedTime timer = new ElapsedTime();

    protected boolean onRed = false;
    protected boolean parkAgainstBridge = false;
    protected int skystonePosition;
    protected double autoWait;

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
    protected void initAuto(boolean visionAuto, double robotX, double robotY, double robotTheta) {
        // Initialize all the hardware
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);
        drive.softBrakeMotors();
        intake = new IntakeMech(robot);
        foundationGrabbers = new FoundationGrabbers(robot);
        lift = new LiftMech(robot);
        purePursuit = new PurePursuit(robot);

        // Keep hardware from unintentionally moving around
        lift.openClose();
        lift.extendRetract();
        foundationGrabbers.grab();

        // Reset robot position to a specified value
        robot.resetOdometry(robotX, robotY, robotTheta);

        // Setup the phone camera for OpenCV
        if (visionAuto) {
            // Set the phone camera for OpenCV
            phoneCamera = new OpenCvInternalCamera(
                    // Sets if using front or back of camera
                    OpenCvInternalCamera.CameraDirection.FRONT,
                    // ID of the camera monitor relative to the app context
                    hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                            "id", hardwareMap.appContext.getPackageName())
            );

            // Start connection to camera
            phoneCamera.openCameraDevice();

            // Sets the current image processing pipeline to detect the skystone
            OpenCvPipeline detector = new SkystonePipeline();
            phoneCamera.setPipeline(detector);

            // Start the camera streaming and set the phones rotation
            // In this case the phone is going to be upright (portrait)
            phoneCamera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        }

        boolean dpadUpPressed = false, dpadDownPressed = false;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x || gamepad2.x) {
                onRed = false;
                if (robot.y < 0)
                    robot.y *= -1;
            }
            if (gamepad1.b || gamepad2.b) {
                onRed = true;
                if (robot.y > 0)
                    robot.y *= -1;
            }
            if (gamepad1.y || gamepad2.y)
                parkAgainstBridge = true;
            if (gamepad1.a || gamepad2.a)
                parkAgainstBridge = false;

            if (gamepad1.dpad_up || gamepad2.dpad_up && !dpadUpPressed)
                autoWait += .5;
            if (gamepad1.dpad_down || gamepad2.dpad_down && !dpadDownPressed)
                autoWait -= .5;
            dpadUpPressed = gamepad1.dpad_up || gamepad2.dpad_up;
            dpadDownPressed = gamepad1.dpad_down || gamepad2.dpad_down;


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
            telemetry.addData("Wait time before auto starts", autoWait);
            telemetry.addLine("==========");

            if (visionAuto) {
                if (screenPosition.x > 145)
                    skystonePosition =
                            (screenPosition.x < 190) ? 2 : 1;
                else
                    skystonePosition = 3;
                telemetry.addData("Block Selected", skystonePosition);
            }
            telemetry.addLine("==========");

            updateOdometryTelemetry();
        }

        // End streaming for efficiency
        if (visionAuto)
            phoneCamera.stopStreaming();
    }

    protected void initAuto(boolean visionAuto) {
        initAuto(visionAuto, 0, 0, 0);
    }

    /**
     * Does all the setting up for the gyro initialization and waits for opmode to start
     */
    protected void initGyroscope(boolean visionAuto) {
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

        while (!robot.imu.isGyroCalibrated() && !isStopRequested()) {
            telemetry.addData("imu calibration", robot.imu.isGyroCalibrated());
            telemetry.update();
        }

        initAuto(visionAuto, 0, 0, 0);
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
     * Does a point turn to reach a specific angle
     *
     * @param angle     Angle in radians to turn to
     * @param threshold Threshold the the robot angle must be within
     */
    protected void pointTurnToAngle(double angle, double threshold) {
        while (!inThreshold(angle, robot.theta, threshold)) {
            double movementTurn = robot.theta - angle;
            if (movementTurn > Math.PI)
                movementTurn = Math.PI - movementTurn;
            else if (movementTurn < -Math.PI)
                movementTurn = -movementTurn - Math.PI;

            // Cap the turning speed
            if (movementTurn > 1.5)
                movementTurn = 1.5;
            else if (movementTurn < -1.5)
                movementTurn = -1.5;

            // If the robot is sufficiently close to the goal angle increase the speed to allow the robot to hit the angle
            if (Math.abs(movementTurn) < .5)
                movementTurn *= 1.1;

            drive.drive(0, 0, movementTurn);
        }
    }


    /**
     * Displays position data for the end of autonomous
     */
    protected void displayEndAuto() {
        ThreadPool.renewPool();

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
     * @param checkPoint    Checkpoint at which to stop running path
     * @param path          Pure Pursuit path to run the robot along
     * @param P             Proportional for PID
     * @param I             Integral for PID
     * @param D             Derivative for PID
     * @param lookAheadDist Radius of Pure Pursuit look ahead circle
     * @param speed         Max speed the robot will follow the path at
     * @param turnSpeed     Max speed the robot will curve at while following the path
     */
    public void runPurePursuitPath(CheckPoint checkPoint, ArrayList<WayPoint> path,
                                   double P, double I, double D,
                                   double lookAheadDist, double speed, double turnSpeed) {
        // Flips the path to work for red alliance
        if (onRed)
            flipPurePursuitPath(checkPoint, path);

        // Initializes the pure pursuit path and declares the followPath() arguments
        purePursuit.initPath(path, P, I, D);
        // Submits the checkpoint to the thread pool
        pool.submit(checkPoint);
        // Continuously runs the pure pursuit path until the robot hits the checkpoint
        while (opModeIsActive() && !checkPoint.isHit) {
            telemetry.addData("PID", purePursuit.pos.getPID());
            updateOdometryTelemetry();

            purePursuit.followPath(path, lookAheadDist, speed, turnSpeed);
        }
        // Stop the robot and expel the checkpoint form the thread pool
        drive.hardBrakeMotors();
        checkPoint.terminate();

        // Flips the path back to its original
        if (onRed)
            flipPurePursuitPath(checkPoint, path);
    }

    public void runPurePursuitPath(CheckPoint cp, ArrayList<WayPoint> path,
                                   double lookAheadDist, double speed, double turnSpeed) {
        runPurePursuitPath(cp, path, .06, .005, .05, lookAheadDist, speed, turnSpeed);
    }

    public void runPurePursuitPath(CheckPoint cp, ArrayList<WayPoint> path) {
        runPurePursuitPath(cp, path, .06, .005, .05, 4, 1.5, 1);
    }

    // =======
    // PARKING
    // =======

    public CheckPoint
            cp_parkWall = new CheckPoint(9, 81, 2, robot),
            cp_parkBridge = new CheckPoint(21, 81, 2, robot);

    public ArrayList<WayPoint>
            wp_parkWallFromLeft = new ArrayList<>(
            Arrays.asList(
                    new WayPoint(9, 72, 3 * Math.PI / 2),
                    new WayPoint(9, 81, 3 * Math.PI / 2)
            )),
            wp_parkWallFromRight = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(9, 90, Math.PI / 2),
                            new WayPoint(9, 81, Math.PI / 2)
                    )),
            wp_parkBridgeFromLeft = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(21, 72, 3 * Math.PI / 2),
                            new WayPoint(21, 81, 3 * Math.PI / 2)
                    )),
            wp_parkBridgeFromRight = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(21, 90, Math.PI / 2),
                            new WayPoint(21, 81, Math.PI / 2)
                    ));

    // ===============
    // FOUNDATION AUTO
    // ===============

    public CheckPoint
            cp_foundationGrab = new CheckPoint(31.5, 12, 2, robot),
            cp_foundationPull = new CheckPoint(0, 4, 4, robot),
            cp_foundationPush = new CheckPoint(20, 36, 1, robot),
            cp_foundationParkWall = new CheckPoint(-1, 70, 2, robot);

    public ArrayList<WayPoint>
            wp_foundationGrab = new ArrayList<>(
            Arrays.asList(
                    new WayPoint(32, 12, 0)
            )),
            wp_foundationPull = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(0, 12, 0)
                    )),
            wp_foundationPush = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(-1, 48, 0),
                            new WayPoint(20, 36, 0)
                    )),
            wp_foundationParkWall = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(1, 45, 0),
                            new WayPoint(-1, 70, 0)
                    ));

    // =============
    // SKYSTONE AUTO
    // =============

    public CheckPoint
            cp_grabSkystone1_pos1 = new CheckPoint(42, 100, 1, robot),
            cp_grabSkystone2_pos1 = new CheckPoint(48, 130, 1, robot),
            cp_grabSkystone1_pos2 = new CheckPoint(42, 91, 1, robot),
            cp_grabSkystone2_pos2 = new CheckPoint(42, 115, 1, robot),
            cp_grabSkystone1_pos3 = new CheckPoint(36, 82, 1, robot),
            cp_grabSkystone2_pos3 = new CheckPoint(39, 115.5, 1, robot),
            cp_prepareForDeposit = new CheckPoint(15, 97, 2, robot),
            cp_prepareForDepositPos1 = new CheckPoint(15, 97, 3, robot),
            cp_depositSkystone = new CheckPoint(33, 57, 2, robot),
            cp_prepareForSecondSkystone = new CheckPoint(15, 97, 3, robot),
            cp_prepareForDeposit_2 = new CheckPoint(15, 97, 2, robot),
            cp_prepareForDepositPos1_2 = new CheckPoint(15, 97, 3, robot),
            cp_depositSkystone_2 = new CheckPoint(33, 57, 2, robot);

    public ArrayList<WayPoint>
            wp_grabSkystone1_pos1 = new ArrayList<>(
            Arrays.asList(
                    new WayPoint(42, 100, 3.7637)
            )),
            wp_grabSkystone2_pos1 = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(48, 130, 3 * Math.PI / 2)
                    )),
            wp_grabSkystone1_pos2 = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(42, 91, 3.7637)
                    )),
            wp_grabSkystone2_pos2 = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(42, 115, 3.7637)
                    )),
            wp_grabSkystone1_pos3 = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(36, 82, 3.7637)
                    )),
            wp_grabSkystone2_pos3 = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(39, 115.5, 3.955)
                    )),
            wp_prepareForDeposit = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(15, 97, Math.PI)
                    )
            ),
            wp_depositSkystone = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(24, 106, Math.PI / 2),
                            new WayPoint(33, 57, Math.PI / 2)
                    )
            ),
            wp_prepareForSecondSkystone = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(24, 97, 3 * Math.PI / 2),
                            new WayPoint(15, 97, 3 * Math.PI / 2)
                    ));
}