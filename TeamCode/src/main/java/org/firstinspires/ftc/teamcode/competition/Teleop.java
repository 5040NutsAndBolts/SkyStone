package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * It's teleop... yeah
 */
@TeleOp(name="Teleop", group="Teleop")
public class Teleop extends OpMode {

    private Hardware robot;
    private MecanumDrive driveTrain;
    IntakeMech intake;
    CapStoneMech capstone;

    Arm arm;

    boolean toggle = false;
    boolean aPressed=false;

    /**
     * Instantiates objects
     */
    public Teleop() {
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);
        intake = new IntakeMech(robot);
        capstone = new CapStoneMech(robot);
        arm = new Arm(robot);
    }

    /**
     * Method run on init to initialize hardware
     */
    public void init() {
        // Hardware map setup
        robot.init(hardwareMap);

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

    }

    /**
     * Loop during init phase
     * Tells you if the gyro is calibrated
     */
    @Override
    public void init_loop() {
        telemetry.addData("imu calabration", robot.imu.isGyroCalibrated());
    }

    /**
     * The loop played during the game
     */
    @Override
    public void loop() {

        if(gamepad2.a&&!aPressed)
        {

            aPressed=true;
            toggle=!toggle;

        }
        else if(!gamepad2.a)
        {
            aPressed=false;
        }

        arm.moveWithPid(toggle);

        if(gamepad2.right_trigger>.01)
            capstone.moveSlides(gamepad2.right_trigger);
        else
            capstone.moveSlides(-gamepad2.left_trigger);

        if(gamepad1.right_trigger>.01)
            intake.intakePower(gamepad1.right_trigger);
        else
            intake.intakePower(-gamepad1.left_trigger);
        if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0) {
            driveTrain.brakeMotors();
        } else {
            driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
        telemetry.addData("A: ", gamepad2.a);
        telemetry.addData("Pressed: ", aPressed);
        telemetry.addData("Toggle: ", toggle);
        telemetry.addData("Power: ", Arm.up.getPID());
        telemetry.addData("p: ", Arm.up.p());
        telemetry.addData("i: ", Arm.up.i());
        telemetry.addData("d: ", Arm.up.d());
        telemetry.update();
    }
}
