package org.firstinspires.ftc.teamcode.competition.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.competition.hardware.GrabbingMech;
import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;

@Autonomous(group="Auto",name = "Move the foundation")
public class FoundationMover extends LinearOpMode {

    protected Hardware robot = new Hardware();
    protected MecanumDrive drive = new MecanumDrive(robot);
    protected GrabbingMech grabbingMech = new GrabbingMech(robot);

    private boolean movingRight = false;
    private boolean parkOnWall = true;
    private double waitTime = 0;
    private boolean dPadPressed = false;
    private boolean onBlueAlliance = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // Lets us move the robot during auto (disables brakes)
        drive.softBrakeMotors();

        while (!isStarted()) {
            robot.updatePositionRoadRunner();
            if (onBlueAlliance)
                telemetry.addLine("Players are on blue alliance");
            else
                telemetry.addLine("Players are on red alliance");
            if (movingRight)
                telemetry.addLine("Robot will move right (relative to human)");
            else
                telemetry.addLine("Robot will move left (relative to human)");
            telemetry.addLine("==========");
            if (parkOnWall)
                telemetry.addLine("Robot will park on wall");
            else
                telemetry.addLine("Robot will NOT park on wall");
            telemetry.addLine("==========");
            telemetry.addData("Wait time (in seconds)", waitTime);
            telemetry.addLine("==========");
            telemetry.addData("X Position", robot.x);
            telemetry.addData("Y Position", robot.y);
            telemetry.addData("Rotation", robot.theta);
            telemetry.update();

            if (gamepad1.y)
                parkOnWall = false;
            if (gamepad1.a)
                parkOnWall = true;
            if (gamepad1.x)
                movingRight = false;
            if (gamepad1.b)
                movingRight = true;
            if (gamepad1.dpad_up && !dPadPressed) {
                waitTime += .25;
                dPadPressed = true;
            }
            if (gamepad1.dpad_down && !dPadPressed) {
                waitTime -= .25;
                dPadPressed = true;
            }
            if (!(gamepad1.dpad_down || gamepad1.dpad_up))
                dPadPressed = false;
            if (gamepad1.dpad_left)
                onBlueAlliance = true;
            if (gamepad1.dpad_right)
                onBlueAlliance = false;

            // Updating robot position
            if (gamepad1.back) {
                robot.resetOdometry();
                if (onBlueAlliance) {
                    robot.theta = -3 * Math.PI / 2.0;
                }
                else {
                    robot.theta = 0;
                }
            }

            if (waitTime > 25)
                waitTime = 25;
            if (waitTime < 0)
                waitTime = 0;
        }
        // Re-enables brakes
        drive.hardBrakeMotors();

        // Moving the claw out of the way
        long endTime = System.currentTimeMillis() + 270;
        long dropTime = System.currentTimeMillis() + 200;
        while(System.currentTimeMillis() < endTime && opModeIsActive()) {
            if(System.currentTimeMillis() > dropTime) {
                robot.clawLeft.setPosition(.7);
                robot.clawRight.setPosition(.7);
            }
            robot.towerArmMotor.setPower(.7);
        }
        endTime = System.currentTimeMillis() + 270;
        while(System.currentTimeMillis() < endTime && opModeIsActive())
            robot.towerArmMotor.setPower(-.7);

        // Waiting before the robot actually does anything
        waitTime *= 1000;
        endTime = System.currentTimeMillis() + (int)waitTime;
        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            telemetry.addData("Time left", endTime-System.currentTimeMillis());
            telemetry.update();
        }

        // Move the robot to be vertically aligned with the foundation
        while (robot.x < 100 && opModeIsActive()) {
            robot.updatePositionRoadRunner();
            telemetry.addData("X Position", robot.x);
            telemetry.addData("Y Position", robot.y);
            telemetry.addData("Rotation", robot.theta);
            telemetry.update();

            drive.drive(-.5,0,0);
        }

        // Move the robot to be next to the foundation
        while (robot.y < 27.5 && opModeIsActive()) {
            robot.updatePositionRoadRunner();
            telemetry.addData("X Position", robot.x);
            telemetry.addData("Y Position", robot.y);
            telemetry.addData("Rotation", robot.theta);
            telemetry.update();

            //drive.driveSideWays(-.4,0);
        }

        // Grab the foundation
        endTime = System.currentTimeMillis() + 1000;
        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            drive.drive(0,0,0);
            grabbingMech.grabFoundation();
        }

        // Pull the foundation backwards
        while (robot.y > 0 && opModeIsActive()) {
            robot.updatePositionRoadRunner();
            telemetry.addData("X Position", robot.x);
            telemetry.addData("Y Position", robot.y);
            telemetry.addData("Rotation", robot.theta);
            telemetry.update();

            drive.drive(0,.5,0);
        }

        // Release the foundation
        endTime = System.currentTimeMillis() + 1000;
        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            drive.drive(0,0,0);
            grabbingMech.resetFoundation();
        }

        // Move the robot to the parking position
        while (robot.x > 69 && opModeIsActive()) {
            robot.updatePositionRoadRunner();
            telemetry.addData("X Position", robot.x);
            telemetry.addData("Y Position", robot.y);
            telemetry.addData("Rotation", robot.theta);
            telemetry.update();

            drive.drive(.6,0,0);
        }

        // Move to park against the sky bridge
        if (!parkOnWall) {
            while (robot.y < 28 && opModeIsActive()) {
                robot.updatePositionRoadRunner();
                telemetry.addData("X Position", robot.x);
                telemetry.addData("Y Position", robot.y);
                telemetry.addData("Rotation", robot.theta);
                telemetry.update();

                drive.drive(0,-.4,0);
            }
        }

        drive.drive(0,0,0);

    }
}
