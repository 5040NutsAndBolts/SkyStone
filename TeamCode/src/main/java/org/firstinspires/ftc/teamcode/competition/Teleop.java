package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.hardware.*;


@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode {

    private Hardware robot;
    private MecanumDrive drive;
    private IntakeMech intake;
    private FoundationGrabbers foundationGrabbers;

    private boolean intakeReleased = false;

    /**
     * Instantiates objects
     */
    public Teleop() {
        robot = new Hardware();
        drive = new MecanumDrive(robot);
        intake = new IntakeMech(robot);
        foundationGrabbers = new FoundationGrabbers(robot);
    }

    /**
     * Method run on init to initialize hardware
     */
    public void init() {
        robot.init(hardwareMap);
    }

    /**
     * The loop played during the game
     */
    @Override
    public void loop() {
        intake.releaseIntake();

        robot.updatePositionRoadRunner();
        telemetry.addData("Intake speed", intake.intakeSpeed);
        telemetry.addLine("=========");
        telemetry.addData("X Position", robot.x);
        telemetry.addData("Y Position", robot.y);
        telemetry.addData("Rotation", robot.theta);
        telemetry.update();

        // ==========
        // Top Half (Gamepad 2)
        // ==========

        // Controlling the speed of the intake
        if (gamepad2.b) {
            intake.intakeSpeed = .25;
        } else if (gamepad2.a) {
            intake.intakeSpeed = .5;
        } else {
            intake.intakeSpeed = 1;
        }

        if (gamepad2.left_bumper)
            foundationGrabbers.release();
        if (gamepad2.right_bumper)
            foundationGrabbers.grab();

        // ==========
        // Bottom Half (Gamepad 1)
        // ==========

        // Intake/Outtake
        if (gamepad1.right_trigger > .01)
            intake.setPower(-1);
        else if (gamepad1.left_trigger > .01)
            intake.setPower(1);
        else
            intake.setPower(0);

        // Drive Train
        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0)
            drive.hardBrakeMotors();
        else
            drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
    }
}
