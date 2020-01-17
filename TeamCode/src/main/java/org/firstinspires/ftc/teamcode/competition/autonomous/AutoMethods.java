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
import static org.firstinspires.ftc.teamcode.competition.helperclasses.HelperMethods.angleWrap;
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

        if (onRed)
            robot.resetOdometry(robot.x, -robot.y, robot.theta + Math.PI);

        // Keep hardware from unintentionally moving around
        lift.openClaw();
        lift.retractClaw();
        foundationGrabbers.release();
        robot.intakeBlock.setPosition(1);

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
                if (onRed)
                    robot.resetOdometry(-robot.x, robot.y, robot.theta);
                onRed = false;
            }
            if (gamepad1.b || gamepad2.b) {
                if (!onRed)
                    robot.resetOdometry(-robot.x, robot.y, robot.theta);
                onRed = true;
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

            if (visionAuto && !onRed) {
                if (screenPosition.x < 90)
                    skystonePosition = 3;
                else if (screenPosition.x < 145)
                    skystonePosition = 2;
                else
                    skystonePosition = 1;
                telemetry.addData("Block Selected", skystonePosition);
            }
            else if (visionAuto && onRed) {
                if (screenPosition.x < 75)
                    skystonePosition = 1;
                else if (screenPosition.x < 100)
                    skystonePosition = 2;
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
        while (!inThreshold(angle, robot.theta, threshold) && opModeIsActive()) {
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
    private void flipPath(boolean red, CheckPoint checkPoint, ArrayList<WayPoint> wayPoints) {
        checkPoint.y *= -1;
        if (red)
            for (int i = 0; i < wayPoints.size(); i++) {
                wayPoints.set(i, new WayPoint(wayPoints.get(i).x, -wayPoints.get(i).y, angleWrap(wayPoints.get(i).angle - Math.PI)));
            }
        else
            for (int i = 0; i < wayPoints.size(); i++) {
                wayPoints.set(i, new WayPoint(wayPoints.get(i).x, -wayPoints.get(i).y, angleWrap(wayPoints.get(i).angle + Math.PI)));
            }
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
            flipPath(true, checkPoint, path);

        // Initializes the pure pursuit path and declares the followPath() arguments
        purePursuit.initPath(path, P, I, D);
        // Submits the checkpoint to the thread pool
        pool.submit(checkPoint);
        // Start a timer to count down so the robot doesn't take too long getting to a checkpoint
        timer.reset();
        timer.startTime();
        // Continuously runs the pure pursuit path until the robot hits the checkpoint
        while (opModeIsActive() && !checkPoint.isHit) {
            telemetry.addData("PID", purePursuit.pos.getPID());
            updateOdometryTelemetry();

            purePursuit.followPath(path, lookAheadDist, speed, turnSpeed);

            // If its been more than 5 seconds and the checkpoint hasn't been hit, just say it has
            if (timer.seconds() >= 6) {
                checkPoint.isHit = true;
                checkPoint.onHit();
                checkPoint.terminate();
            }
        }
        // Stop the robot and expel the checkpoint form the thread pool
        drive.hardBrakeMotors();
        checkPoint.terminate();

        // Flips the path back to its original
        if (onRed)
            flipPath(false, checkPoint, path);
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
            // Parking against the wall
            cp_parkWall = new CheckPoint(8, 70, 2, robot),
            // Parking against the neutral bridge
            cp_parkBridge = new CheckPoint(36, 70, 2, robot);

    public ArrayList<WayPoint>
            // Robot quickly rotates then moves next to the wall and slides down
            wp_parkWallFromLeft = new ArrayList<>(
            Arrays.asList(
                    new WayPoint(8, 60, Math.PI / 2),
                    new WayPoint(8, 70, Math.PI / 2)
            )),
            wp_parkWallFromRight = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(8, 90, Math.PI / 2),
                            new WayPoint(8, 70, Math.PI / 2)
                    )),
            // Robot quickly rotates then moves up to near the neutral bridge and over to park
            wp_parkBridgeFromLeft = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(38, 72, 3 * Math.PI / 2),
                            new WayPoint(36, 70, 3 * Math.PI / 2)
                    )),
            wp_parkBridgeFromRight = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(20, 90, 3 * Math.PI / 2),
                            new WayPoint(20, 70, 3 * Math.PI / 2)
                    ));

    // ===============
    // FOUNDATION AUTO
    // ===============

    public CheckPoint
            cp_foundationGrab = new CheckPoint(32, 11, 2, robot),
            cp_foundationPushImproved = new CheckPoint(35, 15, 2, robot),
            cp_foundationParkPrep = new CheckPoint(40, 44, 2, robot),
            cp_foundationParkWall = new CheckPoint(0, 70, 2, robot),
            cp_foundationParkBridge = new CheckPoint(21, 75, 2, robot);

    public ArrayList<WayPoint>
            wp_foundationGrab = new ArrayList<>(
            Arrays.asList(
                    new WayPoint(32, 11, 0)
            )),
            wp_foundationPushImproved = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(0, 44, 0),
                            new WayPoint(45,44,0),
                            new WayPoint(35,15,0)
                    )),
            wp_foundationParkPrep = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(40, 44, 0)
                    )),
            wp_foundationParkWall = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(-1,44,0),
                            new WayPoint(0,70,0)
                    )),
            wp_foundationParkBridge = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(20,44,0),
                            new WayPoint(30,70,0)
                    ));

    // =============
    // SKYSTONE AUTO
    // =============

    public CheckPoint
            // Position to grab the right most skystone closest to the sky bridge
            cp_grabSkystone1_pos1 = new CheckPoint(52-0.7874016, 96, 3, robot),
            // Position to grab the right most skystone closest to the wall
            cp_grabSkystone2_pos1 = new CheckPoint(52-0.7874016, 122, 1.5, robot),

            // Position to grab the middle skystone closest to the sky bridge
            cp_grabSkystone1_pos2 = new CheckPoint(52-0.7874016, 89, 1.2, robot),
            // Position to grab the middle skystone closest to the wall
            cp_grabSkystone2_pos2 = new CheckPoint(53-0.7874016, 113, 1.3, robot),

            // Position to grab the left most skystone closest to the sky bridge
            cp_grabSkystone1_pos3 = new CheckPoint(44.5, 98, 2, robot),
            // Position to grab the left most skystone closest to the wall
            cp_grabSkystone2_pos3 = new CheckPoint(53-0.7874016, 104, 3, robot),

            // Backing up and preparing to deposit the first skystone
            cp_prepareForDeposit = new CheckPoint(19, 97, 3, robot),

            // Driving and depositing the skystone
            cp_depositSkystone = new CheckPoint(33, 57, 2, robot),
            cp_depositSkystone_2 = new CheckPoint(33, 57, 2, robot),

            // Backing up and preparing to deposit the second skystone
            cp_prepareForSecondSkystone = new CheckPoint(19, 97, 3, robot);

    public ArrayList<WayPoint>
            // Right most skystone paths
            wp_grabSkystone1_pos1 = new ArrayList<>(
            Arrays.asList(
                    new WayPoint(30-0.7874016, 90-0.7874016, 3 * Math.PI / 2),
                    new WayPoint(52-0.7874016, 96-0.7874016, 3 * Math.PI / 2)
            )),
            wp_grabSkystone2_pos1 = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(40-0.7874016, 120, 3 * Math.PI / 2),
                            new WayPoint(52-0.7874016, 122, 3 * Math.PI / 2)
                    )),
            // Middle skystone paths
            wp_grabSkystone1_pos2 = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(9-0.7874016, 95, 3 * Math.PI / 2),
                            new WayPoint(53-0.7874016, 89, 3 * Math.PI / 2)
                    )),
            wp_grabSkystone2_pos2 = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(73, 133, 3 * Math.PI / 2),
                            new WayPoint(52, 113, 3 * Math.PI / 2)
                    )),
            // Left most skystone paths
            wp_grabSkystone1_pos3 = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(18-0.7874016, 110, 2.3),
                            new WayPoint(44.5, 97, 2.2)
                    )),
            wp_grabSkystone2_pos3 = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(53-0.7874016, 104, 3 * Math.PI / 2)
                    )),
            // Depositing the skystones
            wp_prepareForDeposit = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(19, 97, Math.PI)
                    )
            ),
            wp_depositSkystonePos3 = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(24, 106, 3 * Math.PI / 2),
                            new WayPoint(33, 57, 3 * Math.PI / 2)
                    )
            ),
            wp_depositSkystone = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(24, 106, 3 * Math.PI / 2),
                            new WayPoint(33, 57, 3 * Math.PI / 2)
                    )
            ),
            // Moving to grab the second skystone
            wp_prepareForSecondSkystone = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(19, 97, 3 * Math.PI / 2)
                    ));

    // ============================
    // SKYSTONE AND FOUNDATION AUTO
    // ============================

    public CheckPoint
            // Position to grab the left most skystone closest to the wall
            cp_foundationGrabSkystone2_pos3 = new CheckPoint(55, 104, 3, robot),
            // run to the foundation with the skystone
            cp_skystoneToFoundation = new CheckPoint(44, 10, 2, robot),
            cp_skystoneToFoundation_2 = new CheckPoint(24, 40, 2, robot);

    public ArrayList<WayPoint>
            wp_foundationGrabSkystone2_pos3 = new ArrayList<>(
            Arrays.asList(
                    new WayPoint(56, 104, 3 * Math.PI / 2)
            )),
            wp_skystoneToFoundation = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(30, 35, 0),
                            new WayPoint(44, 10, 0)
                    )),
            wp_skystoneToFoundation_2 = new ArrayList<>(
                    Arrays.asList(
                            new WayPoint(20, 32, 0),
                            new WayPoint(32, 32, 0),
                            new WayPoint(24, 40, 0)
                    ));
}