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
            // Move to align with middle of foundation
            while (robot.y < 11 && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(0,-.3,0);
            }
            waitTime(.1);

            // Move forward to grab foundation
            while (robot.x > -30.5 && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(.5,0,0);
            }
            waitTime(.1);

            // Grab the foundation
            grabbingMech.grabFoundation();
            waitTime(.75);

            // Pull the foundation back
            long endTime = System.currentTimeMillis() + 3500;
            while(System.currentTimeMillis() < endTime && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(-.7,0,0);
            }
            waitTime(.1);

            // Release the foundation
            grabbingMech.resetFoundation();
            waitTime(.75);

            // Drive horizontal to the foundation
            while (robot.y > -18 && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(.15,.4,0);
            }
            waitTime(.1);

            // Move in front of the foundation
            while(robot.x > -12 && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(.5,0,0);
            }
            waitTime(.1);

            // Push the foundation against the wall
            endTime = System.currentTimeMillis() + 2500;
            while(System.currentTimeMillis() < endTime && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(0,-.5,0);
            }
            waitTime(.1);

            // If parking against wall
            if (parkAgainstWall) {
                // Move back to touch wall
                endTime = System.currentTimeMillis() + 4500;
                while(System.currentTimeMillis() < endTime && opModeIsActive()) {
                    updateOdometryTeleop();
                    drive.drive(-.4,0,0);
                }
            }

            // If parking against sky bridge
            else {
                // Move forward slightly
                while(robot.x > -30 && opModeIsActive()) {
                    updateOdometryTeleop();
                    drive.drive(.3,0,0);
                }
            }
            waitTime(.1);

            // Move the foundation moves back to inside the robot
            grabbingMech.grabFoundation();
            waitTime(.75);

            // Move under sky bridge
            while(robot.y > -36 && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(0,.3,0);
            }
            drive.hardBrakeMotors();
        }
        // Inverse for red alliance
        else {
            // Move to align with middle of foundation
            while (robot.y > -11 && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(0,.3,0);
            }
            waitTime(.1);

            // Move forward to grab foundation
            while (robot.x > -30.5 && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(.5,0,0);
            }
            waitTime(.1);

            // Grab the foundation
            grabbingMech.grabFoundation();
            waitTime(.75);

            // Pull the foundation back
            long endTime = System.currentTimeMillis() + 3500;
            while(System.currentTimeMillis() < endTime && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(-.7,0,0);
            }
            waitTime(.1);

            // Release the foundation
            grabbingMech.resetFoundation();
            waitTime(.75);

            // Drive horizontal to the foundation
            while (robot.y < 18 && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(.15,-.4,0);
            }
            waitTime(.1);

            // Move in front of the foundation
            while(robot.x > -12 && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(.5,0,0);
            }
            waitTime(.1);

            // Push the foundation against the wall
            endTime = System.currentTimeMillis() + 2500;
            while(System.currentTimeMillis() < endTime && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(0,.5,0);
            }
            waitTime(.1);

            // If parking against wall
            if (parkAgainstWall) {
                // Move back to touch wall
                endTime = System.currentTimeMillis() + 4500;
                while(System.currentTimeMillis() < endTime && opModeIsActive()) {
                    updateOdometryTeleop();
                    drive.drive(-.4,0,0);
                }
            }

            // If parking against sky bridge
            else {
                // Move forward slightly
                while(robot.x > -30 && opModeIsActive()) {
                    updateOdometryTeleop();
                    drive.drive(.3,0,0);
                }
            }
            waitTime(.1);

            // Move the foundation moves back to inside the robot
            grabbingMech.grabFoundation();
            waitTime(.75);

            // Move under sky bridge
            while(robot.y < 36 && opModeIsActive()) {
                updateOdometryTeleop();
                drive.drive(0,-.3,0);
            }
            drive.hardBrakeMotors();
        }
    }
}
