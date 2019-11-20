package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.competition.Hardware;
import org.firstinspires.ftc.teamcode.competition.drivetrain.MecanumDrive;

/**
 * It's teleop... yeah
 */
@TeleOp(name="Teleop", group="Teleop")
public class Teleop extends OpMode {

    private Hardware robot;
    private MecanumDrive driveTrain;

    private boolean onlyForward = false, onlySideways = false;

    /**
     * Instantiates objects
     */
    public Teleop() {
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);
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

        // Odometry setup
        robot.resetEncoders();

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
        robot.updatePositionRoadRunner();
        telemetry.addData("onlyForward", onlyForward);
        telemetry.addData("onlySideways", onlySideways);
        telemetry.addLine("==========");
        telemetry.addData("xPos", robot.x);
        telemetry.addData("yPos", robot.y);
        telemetry.addData("theta", robot.prevHeading);
        telemetry.addLine("==========");
        telemetry.addData("total left traveled(cm)", robot.rightOdomTraveled);
        telemetry.addData("total right traveled(cm)", robot.leftOdomTraveled);
        telemetry.addData("total center traveled(cm)", robot.centerOdomTraveled);
        telemetry.addLine("==========");
        telemetry.addData("leftEncoder value", robot.prevL);
        telemetry.addData("rightEncoder value", robot.prevR);
        telemetry.addData("centerEncoder value", robot.prevC);
        telemetry.update();

        // Reset robot position = X
        if(gamepad1.x){
            robot.resetPosition();
            robot.resetEncoders();
        }
        if(gamepad1.y){
            onlyForward = true;
            onlySideways = false;
        }
        if(gamepad1.b) {
            onlyForward = false;
            onlySideways = true;
        }
        if(gamepad1.a) {
            onlyForward = false;
            onlySideways = false;
        }

        if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0) {
            driveTrain.brakeMotors();
        } else if (!onlyForward && !onlySideways){
            driveTrain.drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x );
        } else if (onlyForward) {
            driveTrain.drive(gamepad1.left_stick_y, 0, -gamepad1.right_stick_x);
        } else if (onlySideways) {
            driveTrain.drive(0, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
    }
}
