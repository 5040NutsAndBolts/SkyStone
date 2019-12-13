package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group="Auto",name = "Foundation Only")
public class FoundationAuto extends AutoMethods {

    private boolean onBlueAlliance = true;
    private boolean onLeftOfBridge = true;
    private boolean parkAgainstWall = true;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        // Lets us move the robot during auto (disables brakes)
        drive.softBrakeMotors();

        while (!isStarted()) {
            if (onBlueAlliance)
                telemetry.addLine("We are BLUE alliance");
            else
                telemetry.addLine("We are RED alliance");
            telemetry.addLine();
            telemetry.addLine();
            if (parkAgainstWall)
                telemetry.addLine("We are parking against the wall");
            else
                telemetry.addLine("We are NOT parking against the wall");
            telemetry.addLine();
            updateOdometryTeleop();

            if (gamepad1.dpad_left)
                onBlueAlliance = true;
            if (gamepad1.dpad_right)
                onBlueAlliance = false;
            if (gamepad1.x)
                onLeftOfBridge = false;
            if (gamepad1.b)
                onLeftOfBridge = true;
            if (gamepad1.y)
                parkAgainstWall = false;
            if (gamepad1.a)
                parkAgainstWall = true;
        }

        robot.resetOdometry(0,0,0);
        grabbingMech.resetFoundation();

        // Run for blue alliance
        if (onBlueAlliance) {
            // Move forward to grab foundation
            while (robot.y < 16) {
                updateOdometryTeleop();
                drive.drive(.4,0,0);
            }
            waitTime(.2);
            // Grab the foundation
            grabbingMech.grabFoundation();
            waitTime(1.5);
            // Pull the foundation back
            while (robot.y > .1) {
                updateOdometryTeleop();
                drive.drive(-.3, 0, 0);
            }
            waitTime(.2);
            // Release the foundation
            grabbingMech.resetFoundation();
            waitTime(1.5);
            // Drive horizontal to the foundation
            while (robot.x > -16) {
                updateOdometryTeleop();
                drive.drive(0,.3,0);
            }
            waitTime(.2);
            // Move in front of the foundation
            while(robot.y < 12) {
                updateOdometryTeleop();
                drive.drive(.4,0,0);
            }
            waitTime(.2);
            // Push the foundation against the wall
            while(robot.x < -10) {
                updateOdometryTeleop();
                drive.drive(0,-.3,0);
            }
            waitTime(.2);
            // If parking against wall
            if (parkAgainstWall) {
                // Move back to touch wall
                while(robot.y > .1) {
                    updateOdometryTeleop();
                    drive.drive(-.4,0,0);
                }
            }
            // If parking against sky bridge
            else {
                // Move forward slightly
                while(robot.y < 14) {
                    updateOdometryTeleop();
                    drive.drive(.4,0,0);
                }
            }
            waitTime(.2);
            // Move under sky bridge
            while(robot.x > -20) {
                updateOdometryTeleop();
                drive.drive(0,.3,0);
            }
            drive.hardBrakeMotors();
        }
        // Inverse for red alliance
        else {
            // Move forward to grab foundation
            while (robot.y < 16) {
                updateOdometryTeleop();
                drive.drive(.4,0,0);
            }
            waitTime(.2);
            // Grab the foundation
            grabbingMech.grabFoundation();
            waitTime(1.5);
            // Pull the foundation back
            while (robot.y > .1) {
                updateOdometryTeleop();
                drive.drive(-.3, 0, 0);
            }
            waitTime(.2);
            // Release the foundation
            grabbingMech.resetFoundation();
            waitTime(1.5);
            // Drive horizontal to the foundation
            while (robot.x < 16) {
                updateOdometryTeleop();
                drive.drive(0,-.3,0);
            }
            waitTime(.2);
            // Move in front of the foundation
            while(robot.y < 12) {
                updateOdometryTeleop();
                drive.drive(.4,0,0);
            }
            waitTime(.2);
            // Push the foundation against the wall
            while(robot.x > 10) {
                updateOdometryTeleop();
                drive.drive(0,.3,0);
            }
            waitTime(.2);
            // If parking against wall
            if (parkAgainstWall) {
                // Move back to touch wall
                while(robot.y > .1) {
                    updateOdometryTeleop();
                    drive.drive(-.4,0,0);
                }
            }
            // If parking against sky bridge
            else {
                // Move forward slightly
                while(robot.y < 14) {
                    updateOdometryTeleop();
                    drive.drive(.4,0,0);
                }
            }
            waitTime(.2);
            // Move under sky bridge
            while(robot.x < 20) {
                updateOdometryTeleop();
                drive.drive(0,-.3,0);
            }
            drive.hardBrakeMotors();
        }
    }
}
