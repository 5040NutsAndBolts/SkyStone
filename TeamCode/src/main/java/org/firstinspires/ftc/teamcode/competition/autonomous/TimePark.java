package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;

@Autonomous(group="Auto",name = "Time Park")
public class TimePark extends AutoMethods {

    private double waitTime = 0;
    private boolean movingRight = true;
    private boolean dPadPressed = false;
    private boolean parkingAgainstWall = true;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        // Lets us move the robot during auto (disables brakes)
        drive = new MecanumDrive(robot);
        drive.softBrakeMotors();

        while (!isStarted()) {
            if (parkingAgainstWall)
                telemetry.addLine("Parking against WALL");
            else
                telemetry.addLine("Parking against SKYBRIDGE");
            telemetry.addLine("");
            telemetry.addData("Wait time in seconds: ", waitTime);
            telemetry.addLine("");
            if (movingRight)
                telemetry.addLine("Moving RIGHT relative to player");
            else
                telemetry.addLine("Moving LEFT relative to player");
            telemetry.update();

            if (gamepad1.dpad_up && !dPadPressed) {
                waitTime += .25;
                dPadPressed = true;
            }
            if (gamepad1.dpad_down && !dPadPressed) {
                waitTime -= .25;
                dPadPressed = true;
            }

            if (gamepad1.dpad_left) {
                movingRight = false;
                dPadPressed = true;
            }
            if (gamepad1.dpad_right) {
                movingRight = true;
                dPadPressed = true;
            }

            if (!(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right))
                dPadPressed = false;

            if (gamepad1.a)
                parkingAgainstWall = true;
            if (gamepad1.y)
                parkingAgainstWall = false;

            if (waitTime > 25)
                waitTime = 25;
            if (waitTime < 0)
                waitTime = 0;
        }        long endTime = System.currentTimeMillis() + (int)(waitTime * 1000);
        while (opModeIsActive() && System.currentTimeMillis() < endTime);

        if (!parkingAgainstWall) {
            endTime = System.currentTimeMillis() + 1250;
            while (opModeIsActive() && System.currentTimeMillis() < endTime)
                drive.drive(-.4, 0, 0);
        }

        endTime = System.currentTimeMillis() + 1500;
        if (movingRight)
            while (opModeIsActive() && System.currentTimeMillis() < endTime)
                drive.drive(0, -.4, 0);
        else
            while (opModeIsActive() && System.currentTimeMillis() < endTime)
                drive.drive(0, .4, 0);

        drive.hardBrakeMotors();
    }
}
