package org.firstinspires.ftc.teamcode.competition.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.competition.helperclasses.HelperMethods;

@Autonomous(group="Auto",name = "Parking Auto")
public class ParkingOnly extends AutoMethods {

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
            if (gamepad1.dpad_left) {
                onBlueAlliance = true;
                //robot.theta = 0;
            }
            if (gamepad1.dpad_right) {
                onBlueAlliance = false;
                //robot.theta = Math.toRadians(180);
            }

            // Updating robot position
            if (gamepad1.back) {
                robot.resetOdometry();
                if (onBlueAlliance)
                    robot.theta = -3 * Math.PI / 2.0;
                else
                    robot.theta = 0;
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

        // While the robot is not within roughly 3 inches of the destination, drive either left or right
        if (onBlueAlliance) {
            if (movingRight)
                while (robot.x > 69 && opModeIsActive()) {
                    robot.updatePositionRoadRunner();
                    telemetry.addData("X Position", robot.x);
                    telemetry.addData("Y Position", robot.y);
                    telemetry.addData("Rotation", robot.theta);
                    telemetry.update();

                    drive.drive(.6, 0, 0);
                }
            else
                while (robot.x < 56 && opModeIsActive()) {
                    robot.updatePositionRoadRunner();
                    telemetry.addData("X Position", robot.x);
                    telemetry.addData("Y Position", robot.y);
                    telemetry.addData("Rotation", robot.theta);
                    telemetry.update();

                    drive.drive(-.6, 0, 0);
                }
        } else {
            if (!movingRight)
                while (robot.x > 69 && opModeIsActive()) {
                    robot.updatePositionRoadRunner();
                    telemetry.addData("X Position", robot.x);
                    telemetry.addData("Y Position", robot.y);
                    telemetry.addData("Rotation", robot.theta);
                    telemetry.update();

                    drive.drive(.6, 0, 0);
                }
            else
                while (robot.x < 56 && opModeIsActive()) {
                    robot.updatePositionRoadRunner();
                    telemetry.addData("X Position", robot.x);
                    telemetry.addData("Y Position", robot.y);
                    telemetry.addData("Rotation", robot.theta);
                    telemetry.update();

                    drive.drive(-.6, 0, 0);
                }
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

    /**
     * Moves the robot sideways a number of inches (relative to its current position)
     * @param inches Numbers of inches sideways the robot will move
     * @param thresholdPercent How close the robot will get to the target position
     */
    private void sidewaysInch(double inches, double thresholdPercent) {
        double endPosX = robot.x + inches;
        while (opModeIsActive()) {
            if (HelperMethods.inThreshhold(robot.x, endPosX, thresholdPercent)) {
                drive.drive(0,0,0);
                break;
            }
            else if (robot.x < endPosX)
                drive.drive(0,-.5,0);
            else if (robot.x > endPosX)
                drive.drive(0,.5,0);
        }
    }

    /**
     * Moves the robot forwards a number of inches (relative to its current position)
     * @param inches Numbers of inches sideways the robot will move
     * @param thresholdPercent How close the robot will get to the target position
     */
    private void forwardsInch(double inches, double thresholdPercent) {
        double endPosX = robot.y + inches;
        while (opModeIsActive()) {
            if (HelperMethods.inThreshhold(robot.y, endPosX, thresholdPercent)) {
                drive.drive(0,0,0);
                break;
            }
            else if (robot.x > endPosX)
                drive.drive(-.5,0,0);
            else if (robot.x < endPosX)
                drive.drive(.5,0,0);
        }
    }
}
