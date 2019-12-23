package org.firstinspires.ftc.teamcode.competition.hardware;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


/**
 * This class is for setting up all the hardware components of the robot.
 * This will have all the sensors, motors and servos declarations.
 * It will also be used to initialize everything for autonomous
 *
 * Note: 0, 0, 0 on the field is the robot in the blue depot with the intake facing the red depot
 */
public class Hardware {

    public ThreeTrackingWheelLocalizer odom = new ThreeTrackingWheelLocalizer(
            new ArrayList<>(Arrays.asList(new Pose2d(8,0,Math.PI/2),
                    new Pose2d(0,8.49,0),
                    new Pose2d(0,-8.49,0)))) {
        @Override
        public List<Double> getWheelPositions() {
            ArrayList<Double> wheelPositions = new ArrayList<>(3);
            wheelPositions.add(centerOdomTraveled);
            wheelPositions.add(leftOdomTraveled);
            wheelPositions.add(rightOdomTraveled);
            return wheelPositions;
        }
    };

    // Measurements and such kept as variables for ease of use
    // Ticks Per Rotation of an odometry wheel
    private static final double ODOM_TICKS_PER_ROTATION = 1440;
    // Radius of an odometry wheel in cm
    private static final double ODOM_WHEEL_RADIUS = 3.6/2.54;
    // Circumference of an odometry wheel in cm
    private static final double WHEEL_CIRCUM = 2.0 * Math.PI * ODOM_WHEEL_RADIUS;
    // Number of ticks in a centimeter using dimensional analysis
    private static final double ODOM_TICKS_PER_CM = ODOM_TICKS_PER_ROTATION /( WHEEL_CIRCUM);

    // Robot physical location
    // 0, 0, 0 is the blue depot with the intake facing opposite the red depot
    public double x = 0;
    public double y = 0;
    public double theta = 0;

    // Hardware mapping
    private HardwareMap hwMap;

    // Gyro
    public BNO055IMU imu;

    // Drive train
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;

    // Odometry hardware
    private DcMotorEx leftEncoder = null;
    private DcMotorEx rightEncoder = null;
    private DcMotorEx centerEncoder = null;

    // Rev Expansion Hub Data
    public RevBulkData bulkData;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor leftOdom, rightOdom, centerOdom;

    // Odometry encoder positions
    public int leftEncoderPos = 0;
    public int centerEncoderPos = 0;
    public int rightEncoderPos = 0;

    // Real world distance traveled by the wheels
    public double leftOdomTraveled = 0;
    public double rightOdomTraveled = 0;
    public double centerOdomTraveled = 0;

    // Intake
    public DcMotor intakeLeft;
    public DcMotor intakeRight;
    public Servo stoneGuide;

    // Tower arm
    public DcMotor towerArmMotor;
    public Servo clawRight;
    public Servo clawLeft;
    public enum TowerArmPos {
        RAISE,
        LOWER,
        STOP,
        RESET
    }

    // Capstone arm
    public Servo capstonePlacer;
    public DcMotor capstoneSlides;

    // Foundation/Stone grabber
    public Servo grabber;
    public Servo foundationGrabber1;
    public Servo foundationGrabber2;

    /**
     * Simple constructor to set hardware mapping to null
     */
    public Hardware() { hwMap = null; }

    /**
     * Initialization of hardware
     * @param mapping hardware map passed into class
     */
    public void init(HardwareMap mapping){
        hwMap = mapping;

        // Drive train
            // left front
            leftFront = hwMap.get(DcMotorEx.class, "leftFront");
            // Motors don't have encoders on them because we're using odometry
            leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            // When motors aren't receiving power, they will attempt to hold their position
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // left rear
            leftRear = hwMap.get(DcMotorEx.class, "leftRear");
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // right front
            rightFront = hwMap.get(DcMotorEx.class, "rightFront");
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // right rear
            rightRear = hwMap.get(DcMotorEx.class, "rightRear");
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Odometry encoder setup
        leftEncoder = hwMap.get(DcMotorEx.class, "leftFront");
        rightEncoder = hwMap.get(DcMotorEx.class, "rightFront");
        centerEncoder = hwMap.get(DcMotorEx.class, "rightRear");

        // Rev ExpansionHub Bulk Data
        expansionHub = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        leftOdom = (ExpansionHubMotor) hwMap.dcMotor.get("leftFront");
        rightOdom = (ExpansionHubMotor) hwMap.dcMotor.get("rightFront");
        centerOdom = (ExpansionHubMotor) hwMap.dcMotor.get("rightRear");

        // Intake motor setup
            // Left intake
            intakeLeft = hwMap.dcMotor.get("intakeLeft");
            intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Right intake
            intakeRight = hwMap.dcMotor.get("intakeRight");
            intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeRight.setDirection(DcMotor.Direction.REVERSE);
            intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stone guide motor setup
        stoneGuide = hwMap.servo.get("stoneGuide");

        // Tower arm setup
            // arm motor
            towerArmMotor = hwMap.dcMotor.get("towerArm");
            towerArmMotor.setDirection(DcMotor.Direction.REVERSE);
            towerArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            towerArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            towerArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // claw servos
            clawLeft = hwMap.servo.get("clawLeft");
            clawRight = hwMap.servo.get("clawRight");
            clawLeft.setDirection(Servo.Direction.REVERSE);

        // Capstone arm motor setup
            // Capstone place
            capstonePlacer = hwMap.servo.get("capstonePlacer");
            // Capstone slides
            capstoneSlides = hwMap.dcMotor.get("capstoneSlides");
            capstoneSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            capstoneSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Foundation/Stone grabber setup
        grabber = hwMap.servo.get("stoneGrabber");
        foundationGrabber1 = hwMap.servo.get("foundGrabber1");
        foundationGrabber1.setDirection(Servo.Direction.REVERSE);
        foundationGrabber2 = hwMap.servo.get("foundGrabber2");
    }

    /**
     * Method for updating the position of the robot using roadrunner
     */
    public void updatePositionRoadRunner() {
        bulkData = expansionHub.getBulkInputData();

        // Change in the distance (centimeters) since the last update for each odometer
        double deltaLeftDist = -(getDeltaLeftTicks() / ODOM_TICKS_PER_CM) * 1.07;
        double deltaRightDist = -(getDeltaRightTicks() / ODOM_TICKS_PER_CM)* 1.07;
        double deltaCenterDist = -getDeltaCenterTicks() / ODOM_TICKS_PER_CM * 1.07;

        // Update real world distance traveled by the odometry wheels, regardless of orientation
        leftOdomTraveled += deltaLeftDist;
        rightOdomTraveled += deltaRightDist;
        centerOdomTraveled += deltaCenterDist;

        odom.update();
        theta = odom.getPoseEstimate().component3();
        x = odom.getPoseEstimate().component1();
        y = odom.getPoseEstimate().component2();

        resetDeltaTicks();

    }

    private void resetDeltaTicks() {
        leftEncoderPos = bulkData.getMotorCurrentPosition(leftOdom);
        rightEncoderPos = bulkData.getMotorCurrentPosition(rightOdom);
        centerEncoderPos = bulkData.getMotorCurrentPosition(centerOdom);
    }

    private int getDeltaLeftTicks() { return leftEncoderPos - bulkData.getMotorCurrentPosition(leftOdom); }

    private int getDeltaRightTicks() { return rightEncoderPos - bulkData.getMotorCurrentPosition(rightOdom); }

    private int getDeltaCenterTicks() { return centerEncoderPos - bulkData.getMotorCurrentPosition(centerOdom); }

    /**
     * Resets odometry position and values back to default at 0, 0, 0
     */
    public void resetOdometry() {
        x = 0;
        y = 0;
        theta = 0;
        leftOdomTraveled = 0;
        rightOdomTraveled = 0;
        leftOdomTraveled = 0;
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoderPos = 0;
        rightEncoderPos = 0;
        centerEncoderPos = 0;

        // Run mode needs to be reset because encoders and wheels are pointing to the same location
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Resets odometry position and values back to specific values
     * @param x X position to reset encoders to
     * @param y Y position to reset encoders to
     * @param theta Rotational value to reset encoders to
     */
    public void resetOdometry(double x, double y, double theta) {
        odom.setPoseEstimate(new Pose2d(x, y, theta));

        leftOdomTraveled = 0;
        rightOdomTraveled = 0;
        leftOdomTraveled = 0;
        leftEncoderPos = 0;
        rightEncoderPos = 0;
        centerEncoderPos = 0;

        // Resets encoder values then sets them back to run without encoders because wheels and odometry are same pointer
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}