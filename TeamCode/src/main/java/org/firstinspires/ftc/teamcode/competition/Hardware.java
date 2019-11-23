package org.firstinspires.ftc.teamcode.competition;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.usb.RobotUsbModule;

import org.firstinspires.ftc.teamcode.competition.helperclasses.HelperMethods;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * This class is for setting up all the hardware components of the robot.
 * This will have all the sensors, motors and servos declarations.
 * It will also be used to initialize everything for autonomous
 */
public class Hardware {

    public ThreeTrackingWheelLocalizer odom=new ThreeTrackingWheelLocalizer(new ArrayList<Pose2d>(Arrays.asList(new Pose2d(8,0,Math.PI/2),new Pose2d(0,8.5,0),new Pose2d(0,-8.5,0)))) {
        @Override
        public List<Double> getWheelPositions() {
            ArrayList<Double> wheelPositions = new ArrayList<Double>(3);
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
        // Distance from left odometry wheel to the right odometry wheel in cm
    private static final double TRACK_WIDTH = 40.194/0.893856554;
        // Circumference of an odometry wheel in cm
    private static final double WHEEL_CIRCUM = 2.0 * Math.PI * ODOM_WHEEL_RADIUS;
        // Number of ticks in a centimeter using dimensional analysis
    private static final double ODOM_TICKS_PER_CM = ODOM_TICKS_PER_ROTATION /( WHEEL_CIRCUM);

    // Robot physical location
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

        // Drive train motor setup
          // left front
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
            // Motors don't have encoders on them because we're using odometry
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        expansionHub = hwMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        leftOdom = (ExpansionHubMotor) hwMap.dcMotor.get("leftFront");
        rightOdom = (ExpansionHubMotor) hwMap.dcMotor.get("rightFront");
        centerOdom = (ExpansionHubMotor) hwMap.dcMotor.get("rightRear");
    }

    public void updatePositionRoadRunner()
    {

        bulkData = expansionHub.getBulkInputData();

        // Change in the distance (centimeters) since the last update for each odometer
        double deltaLeftDist = -(getDeltaLeftTicks() / ODOM_TICKS_PER_CM)*1.07;
        double deltaRightDist = -(getDeltaRightTicks() / ODOM_TICKS_PER_CM)*1.07;
        double deltaCenterDist = -getDeltaCenterTicks() / ODOM_TICKS_PER_CM*1.07;

        // Update real world distance traveled by the odometry wheels, regardless of orientation
        leftOdomTraveled += deltaLeftDist;
        rightOdomTraveled += deltaRightDist;
        centerOdomTraveled += deltaCenterDist;

        odom.update();
        prevHeading=odom.getPoseEstimate().component3();
        x=odom.getPoseEstimate().component1();
        y=odom.getPoseEstimate().component2();

        resetDeltaTicks();

    }

    /**
     * Update global robot position using line approximation
     * Should be used for very small angle measurements (driving in a near-straight line)
     */
    public void updatePosition() {
        bulkData = expansionHub.getBulkInputData();

        // Change in the distance (centimeters) since the last update for each odometer
        double deltaLeftDist = -(getDeltaLeftTicks() / ODOM_TICKS_PER_CM);
        double deltaRightDist = -(getDeltaRightTicks() / ODOM_TICKS_PER_CM);
        double deltaCenterDist = getDeltaCenterTicks() / ODOM_TICKS_PER_CM;

        // Update real world distance traveled by the odometry wheels, regardless of orientation
        leftOdomTraveled += deltaLeftDist;
        rightOdomTraveled += deltaRightDist;
        centerOdomTraveled += deltaCenterDist;

        // The change in angle of the robot since the last update
        double deltaTheta = (deltaRightDist - deltaLeftDist) / TRACK_WIDTH;
        // Easy variable just to hold the average between the two wheels for forward movement
        double deltaY = (deltaLeftDist + deltaRightDist) / 2.0;
        double deltaX = (deltaCenterDist);

        // The global angle of the robot
        theta = (rightOdomTraveled - leftOdomTraveled) / TRACK_WIDTH;

        // Translates the local movement into global position
        y += deltaY * sin(deltaTheta) + deltaX * cos(deltaTheta);
        x += deltaY * cos(deltaTheta) + deltaX * sin(deltaTheta);

        resetDeltaTicks();
    }

    // Location info for line approximation odometry
    public double prevHeading = 0, prevL = 0, prevR = 0, prevC=0;
    public double axisWidth = TRACK_WIDTH;

    public void roadRunnerUpdate()
    {



    }

    /**
     * Updates global position of robot using line approximation math
     * (i.e. driving in straight lines or anywhere there is a very small delta angle)
     */
    public void updatePositionLineApprox()
    {
        bulkData = expansionHub.getBulkInputData();
        // Get encoder values and previous heading
        double l = getDeltaLeftTicks() / ODOM_TICKS_PER_CM;
        double r = getDeltaRightTicks() / ODOM_TICKS_PER_CM;
        double c = -getDeltaCenterTicks() / ODOM_TICKS_PER_CM-18.75*prevHeading;
        double heading = prevHeading;

        // Calculate encoder deltas
        double lDelta = l - prevL;
        double rDelta = r - prevR;
        double cDelta = c - prevC;

        // Calculate omega
        double hDelta = (rDelta - lDelta) / axisWidth;

        // Approximate position using line approximation method
        x += (lDelta+rDelta)/2 * cos(heading + hDelta)+(cDelta)*sin(heading+hDelta);
        y += (lDelta+rDelta)/2 * sin(heading + hDelta)+(cDelta)*cos(heading+hDelta);

        // Set previous values to current values
        prevL = l;
        prevR = r;
        prevC = c;
        prevHeading = heading + hDelta;
    }

    /**
     * The robot moved in an arch, the left encoder measured a distance l and the right wheel a
     * distance r. It was previously at an angle θ and is now at an angle θ + ω. We want to
     * calculate what the new position of the robot is after this move.
     *
     * The way to calculate the new coordinates is to find the radius R of ICC (Instantaneous
     * Center of Curvature – the point around which the robot is turning) and then rotate x_0​
     * and y_0​ around it.
     */
    public void updatePositionCircleApprox() {
        bulkData = expansionHub.getBulkInputData();

        // Get encoder values and previous heading
        double l = -(getDeltaLeftTicks() / ODOM_TICKS_PER_CM*1.025);
        double r = -(getDeltaRightTicks() / ODOM_TICKS_PER_CM);
        double heading = prevHeading;

        // Calculate encoder deltas
        double lDelta = l - prevL;
        double rDelta = r - prevR;

        // Calculate change in angle
        //ld=-2 rd=-3
        double hDelta = (rDelta - lDelta) / axisWidth;

        // Approximate if straight line or to calculate arc
        if (Math.abs(lDelta - rDelta) < 1e-20){
            x += (rDelta+lDelta)/2 * cos(heading);
            y += (rDelta+lDelta)/2 * sin(heading);
        } else {
            // Calculate radius of ICC
            double R =(axisWidth / 2.0) * (rDelta + lDelta) / (rDelta - lDelta);
            // Calculate position by finding point that is rotated around ICC by heading delta
            x += R * (-sin(heading) + sin(hDelta)*cos(heading)+sin(heading)*cos(hDelta));
            y += R * (cos(heading) -  cos(hDelta)*cos(heading)+sin(heading)*sin(hDelta));

        }

        prevL = l;
        prevR = r;
        prevHeading = heading + hDelta;
    }

    /**
     * New version of update position which uses the same concept as the circle approximation
     * BUT the heading is calculated using the middle odometry wheel instead of gyro/weird math
     */
    public void newUpdatePosition(){
        bulkData = expansionHub.getBulkInputData();

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
     * Resets position of the robot to x=0, y=0, theta=0
     */
    public void resetPosition(){
        prevHeading = 0;
        prevL = 0;
        prevR = 0;

        x = 0;
        y = 0;
        theta = 0;
        leftOdomTraveled = 0;
        rightOdomTraveled = 0;
        centerOdomTraveled = 0;
    }

    /**
     * Resets the encoder values to zero
     */
    public void resetEncoders(){
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
}