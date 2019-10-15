package org.firstinspires.ftc.teamcode.competition;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This class is for setting up all the hardware components of the robot.
 * This will have all the sensors, motors and servos declarations.
 * It will also be used to initialize everything for autonomous
 */
public class Hardware {

    // Hardware mapping
    private HardwareMap hwMap;

    // Gyro
    public BNO055IMU imu;

    // Drive train
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;


    /**
     * Simple constructor to set hardware mapping to null
     */
    public Hardware() {
        hwMap = null;
    }

    /**
     * Initialization of hardware
     *
     * @param mapping hardware map passed into class
     */
    public void init(HardwareMap mapping) {
        hwMap = mapping;

        // Drive train motor setup
        // left front
        leftFront = hwMap.dcMotor.get("leftFront");
        // Motors don't have encoders on them because we're using odometry
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // When motors aren't receiving power, they will attempt to hold their position
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // left rear
        leftRear = hwMap.dcMotor.get("leftRear");
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // right front
        rightFront = hwMap.dcMotor.get("rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // right rear
        rightRear = hwMap.dcMotor.get("rightRear");
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

}