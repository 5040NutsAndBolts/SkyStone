package org.firstinspires.ftc.teamcode.competition.hardware;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This class is for setting up all the hardware components of the robot.
 * This will have all the sensors, motors and servos declarations.
 * It will also be used to initialize everything for autonomous
 */
public class Hardware {
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
    public enum ClawPos {
        OPEN,
        CLOSE,
        STOP
    }

    // Capstone arm
    public Servo capstonePlacer;
    public DcMotor capstoneSlides;

    // Hardware mapping
    private HardwareMap hwMap;

    // Gyro
    public BNO055IMU imu;

    // Drive train
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;


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
        towerArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        towerArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // claw servos
        clawLeft = hwMap.servo.get("clawLeft");
        clawRight = hwMap.servo.get("clawRight");

        // Capstone arm motor setup
        // Capstone place
        capstonePlacer = hwMap.servo.get("capstonePlacer");
        // Capstone slides
        capstoneSlides = hwMap.dcMotor.get("capstoneSlides");
        capstoneSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capstoneSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}