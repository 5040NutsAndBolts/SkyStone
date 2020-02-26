package org.firstinspires.ftc.teamcode.competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.hardware.*;
import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;


@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends LinearOpMode {

    private Hardware robot;
    private MecanumDrive drive;
    private IntakeMech intake;
    private FoundationGrabbers foundationGrabbers;
    private LiftMech lift;
    private CapstoneDropper capstoneDropper;
    private Carriage carriage;

    private boolean
            gamepad1PressedB, gamepad1PressedA, gamepad1PressedY, gamepad1PressedX, gamepad2PressedDPad,
            slowMode = false,
            debugging = false;
    private int currentStackLevel = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Instantiate all objects
        robot = new Hardware();
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);
        intake = new IntakeMech(robot);
        foundationGrabbers = new FoundationGrabbers(robot);
        lift = new LiftMech(robot);

        capstoneDropper = new CapstoneDropper(robot);
        carriage = new Carriage(robot);

        // public void init_loop()
        while (!isStarted()) {
            if (!debugging && gamepad1.y || gamepad2.y)
                debugging = true;
            if (debugging && gamepad1.a || gamepad2.a)
                debugging = false;

            telemetry.addData("debugging", debugging);
            if (debugging) {
                telemetry.addData("X Position", robot.x);
                telemetry.addData("Y Position", robot.y);
                telemetry.addData("Rotation", robot.theta);
            }
            telemetry.update();
        }

        // Initialize servos upon starting teleop
        robot.intakeBlock.setPosition(.5);
        lift.openClaw();
        foundationGrabbers.release();

        while(opModeIsActive()) {
            // Telemetry
            telemetry.addData("Slow mode", slowMode);
            telemetry.addLine();
            telemetry.addData("Intake speed", intake.intakeSpeed);
            telemetry.addLine();
            telemetry.addData("Lift level", currentStackLevel);
            telemetry.addLine();
            telemetry.addData("Carriage encoder?", robot.intakeRight.getCurrentPosition());
            telemetry.addData("Lift State:",lift.currentState);
            telemetry.addLine();
            telemetry.addData("Lift encoder?", robot.intakeLeft.getCurrentPosition());
            telemetry.addData("lift",gamepad2.left_stick_y);
            if (debugging) {
                robot.updatePositionRoadRunner();
                telemetry.addLine();
                telemetry.addLine("==========");
                telemetry.addData("X Position", robot.x);
                telemetry.addData("Y Position", robot.y);
                telemetry.addData("Rotation", robot.theta);
            }
            telemetry.update();

            // ====================
            // Top Half (Gamepad 2) - Lift, Carriage, Claw,Capstone
            // ====================

            // Manual control over the lift
            lift.manual(gamepad2.left_stick_y);
            if (gamepad2.left_stick_y != 0)
                lift.currentState = LiftMech.LiftState.Manual;

            // Automatic control over the lift
            if (!gamepad2PressedDPad && gamepad2.dpad_up)     // Raises to current level then increases level
                lift.moveToLevel(currentStackLevel++);
            if (!gamepad2PressedDPad && gamepad2.dpad_down)   // Takes the lift all the way back down
                lift.moveToLevel(0);
            if (!gamepad2PressedDPad && gamepad2.dpad_left)   // Decreases the current stack level w/o moving the lift
                currentStackLevel--;
            if (!gamepad2PressedDPad && gamepad2.dpad_right)  // Increases the current stack level w/o moving the lift
                currentStackLevel++;
            if (gamepad2.left_bumper)                         // Sets stack level to first level (tower knocked down)
                currentStackLevel = 1;

            // Ensuring the currentStackLevel cannot be too low/high
            currentStackLevel = (int)HelperMethods.clamp(1, currentStackLevel, 15);

            // Resetting the carriage
            if (gamepad2.left_trigger > .05)
                lift.reset();

            // Ensures carriage power cannot be 1 because 100% power is weird for the servo
            carriage.manual(gamepad2.right_stick_y);
            if (gamepad2.right_stick_y != 0)
                carriage.carriageState = Carriage.CarriagePosition.Manual;

            // Extending the carriage
            if (gamepad2.x)
                carriage.extend();
            // Retracting the carriage
            if (gamepad2.b)
                carriage.retract();

            if(gamepad2.right_bumper)
                carriage.setExtendedPosition();

            // Reset carriage encoder
            if (gamepad2.right_trigger > .05)
                carriage.reset();

            // Open/close the claw
            if (gamepad2.a)
                lift.closeClaw();
            if (gamepad2.y)
                lift.openClaw();

            // Release the capstone
            if (gamepad2.back)
                capstoneDropper.dropping = true;

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
            if (!gamepad1PressedX && gamepad1.x)
                slowMode = !slowMode;

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
            if (debugging && gamepad1.back)
                robot.resetOdometry(9, 135, 3 * Math.PI / 2);

            // ===================
            // UPDATE CONTROLLERS
            // ===================

            // Gamepad 1
            gamepad1PressedA = gamepad1.a;
            gamepad1PressedB = gamepad1.b;
            gamepad1PressedY = gamepad1.y;
            gamepad1PressedX = gamepad1.x;

            // Gamepad 2
            gamepad2PressedDPad = gamepad2.dpad_right || gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_down;

            // ======================================================
            // LAST RESORT - DO NOT PRESS UNLESS ABSOLUTELY NECESSARY
            //                  KILL THREAD POOL
            // ======================================================
            if (gamepad1.guide || gamepad2.guide)
                ThreadPool.renewPool();
        }

        // Kill all the threads
        ThreadPool.renewPool();
    }
}
