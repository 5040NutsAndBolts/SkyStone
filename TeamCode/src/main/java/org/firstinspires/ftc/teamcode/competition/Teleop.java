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
    private LiftMech lift;

    private boolean
            gamepad1PressedB, gamepad1PressedA, gamepad1PressedY,
            gamepad2PressedA, gamepad2PressedX;
    private boolean startTeleop;
    private boolean slowMode = false;

    /**
     * Instantiates objects
     */
    public Teleop() {
        robot = new Hardware();
        drive = new MecanumDrive(robot);
        intake = new IntakeMech(robot);
        foundationGrabbers = new FoundationGrabbers(robot);
        lift = new LiftMech(robot);
    }

    /**
     * Method run on init to initialize hardware
     */
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addData("X Position", robot.x);
        telemetry.addData("Y Position", robot.y);
        telemetry.addData("Rotation", robot.theta);
        telemetry.update();
    }

    /**
     * The loop played during the game
     */
    @Override
    public void loop() {
        // Initialize servos upon starting teleop
        if (!startTeleop) {
            robot.intakeBlock.setPosition(.5);
            lift.openClose();
            lift.extendRetract();
            foundationGrabbers.grab();
            startTeleop = true;
        }

        if (lift.clawOpen)
            telemetry.addLine("Claw LOOSE");
        else
            telemetry.addLine("Claw GRABBING");
        telemetry.addLine("==========");
        robot.updatePositionRoadRunner();
        telemetry.addData("X Position", robot.x);
        telemetry.addData("Y Position", robot.y);
        telemetry.addData("Rotation", robot.theta);
        telemetry.update();

        // ====================
        // Top Half (Gamepad 2) - Lift, Claw, Claw Extension
        // ====================

        // Raising the lift mechanism
        lift.raiseLower(gamepad2.left_stick_y);

        // Extending out the claw
        if (!gamepad2PressedA && gamepad2.a)
            lift.openClose();

        // Open/close the claw
        if (!gamepad2PressedX && gamepad2.x)
            lift.extendRetract();

        // =======================
        // Bottom Half (Gamepad 1) - Intake, Drive, Foundation Grabbers
        // =======================

        // Controlling the speed of the intake
        if (!gamepad1PressedA && gamepad1.a)
            intake.intakeSpeed = .25;
        else if (!gamepad1PressedB && gamepad1.b)
            intake.intakeSpeed = .5;
        else if (!gamepad1PressedY && gamepad1.y)
            intake.intakeSpeed = 1;

        // Intake/Outtake
        if (gamepad1.right_trigger > .01)
            intake.setPower(-intake.intakeSpeed);
        else if (gamepad1.left_trigger > .01)
            intake.setPower(intake.intakeSpeed);
        else
            intake.setPower(0);

        // Foundation moving
        if (gamepad1.left_bumper)
            foundationGrabbers.release();
        if (gamepad1.right_bumper)
            foundationGrabbers.grab();

        // Slow mode for the drive train
        slowMode = gamepad1.left_stick_button || gamepad1.right_stick_button;

        // Drive Train
        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0)
            drive.hardBrakeMotors();
        else {
            if (!slowMode)
                drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            else
                drive.drive(-gamepad1.left_stick_y / 4, gamepad1.left_stick_x / 4, -gamepad1.right_stick_x / 4);
        }

        // TeleOp debugging
        if (gamepad1.y)
            robot.resetOdometry(9, 9, 3 * Math.PI / 2);

        // ===================
        // UPDATE CONTROLLERS
        // ===================

        // Gamepad 1
        gamepad1PressedA = gamepad1.a;
        gamepad1PressedB = gamepad1.b;
        gamepad1PressedY = gamepad1.y;

        // Gamepad 2
        gamepad2PressedA = gamepad2.a;
        gamepad2PressedX = gamepad2.x;
    }
}
