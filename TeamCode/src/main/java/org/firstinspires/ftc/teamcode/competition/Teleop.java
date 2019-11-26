package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.competition.hardware.*;

/**
 * It's teleop... yeah
 */
@TeleOp(name="Teleop", group="Teleop")
public class Teleop extends OpMode {

    private Hardware robot;
    private MecanumDrive driveTrain;
    private IntakeMech intake;
    private TowerArm towerArm;
    private CapstoneMech capstoneMech;

    private double intakePower = 1;

    /**
     * Instantiates objects
     */
    public Teleop() {
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);
        intake = new IntakeMech(robot);
        towerArm = new TowerArm(robot);
        capstoneMech = new CapstoneMech(robot);
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
        telemetry.addData("imu calibration", robot.imu.isGyroCalibrated());
        robot.towerArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * The loop played during the game
     */
    @Override
    public void loop() {
        telemetry.addData("Tower Arm Encoder Pos", robot.towerArmMotor.getCurrentPosition());
        telemetry.update();

        // Top Half (Gamepad 2)
            // Raising/lowering the tower arm
                if(gamepad2.right_trigger > .01)
                    towerArm.raiseLower(Hardware.TowerArmPos.RAISE);
                else if (gamepad2.left_trigger > .01)
                    towerArm.raiseLower(Hardware.TowerArmPos.LOWER);
                else
                    towerArm.raiseLower(Hardware.TowerArmPos.STOP);

            // Reset the motor encoder for the tower arm
                if(gamepad2.a)
                    towerArm.raiseLower(Hardware.TowerArmPos.RESET);

            // Opening/closing the tower grabber claw
                if(gamepad2.right_bumper)
                    towerArm.openClose(true);
                else if (gamepad2.left_bumper)
                    towerArm.openClose(false);
                if(gamepad2.x)
                    towerArm.openLeft();
                if(gamepad2.b)
                    towerArm.openRight();

            // Raising the capstone placing mechanism
                if(gamepad2.y)
                    capstoneMech.moveSlidesUp();
                else if (gamepad2.a)
                    capstoneMech.moveSlidesDown();
                else
                    capstoneMech.holdSlides();

        // Bottom Half (Gamepad 1)
            // Intake/Outtake
                if(gamepad1.right_trigger>.01)
                    intake.setPower(-intakePower);
                else if(gamepad1.left_trigger>.01)
                    intake.setPower(intakePower);
                else
                    intake.setPower(0);
                if(gamepad1.y)
                    intakePower = 1;
                if(gamepad1.b)
                    intakePower = .6;
                if(gamepad1.a)
                    intakePower = .35;

            // Drive Train
                if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0)
                    driveTrain.brakeMotors();
                else
                    driveTrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
    }
}
