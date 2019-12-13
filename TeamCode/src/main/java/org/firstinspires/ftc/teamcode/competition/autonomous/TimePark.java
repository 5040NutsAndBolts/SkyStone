package org.firstinspires.ftc.teamcode.competition.autonomous;

public class TimePark extends AutoMethods {

    private int waitTime = 0;
    private boolean movingRight = true;
    private boolean dPadPressed = false;
    private boolean parkingAgainstWall = true;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        // Lets us move the robot during auto (disables brakes)
        drive.softBrakeMotors();

        while (!isStarted()) {
            if (parkingAgainstWall)
                telemetry.addData("Parking against WALL", parkingAgainstWall);
            else
                telemetry.addData("Parking against SKYBRIDGE", parkingAgainstWall);
            telemetry.addLine("");
            telemetry.addData("Wait time in seconds: ", waitTime);
            telemetry.addLine("");
            if (movingRight)
                telemetry.addLine("Moving RIGHT relative to player");
            else
                telemetry.addLine("Moving LEFT relative to player");

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
        }
        long endTime = System.currentTimeMillis() + waitTime * 1000;
        while (opModeIsActive() && System.currentTimeMillis() < endTime);

        endTime = System.currentTimeMillis() + 1000;
        if (movingRight)
            while (opModeIsActive() && System.currentTimeMillis() < endTime)
                drive.drive(0, -.1, 0);
        else
            while (opModeIsActive() && System.currentTimeMillis() < endTime)
                drive.drive(0, .1, 0);
    }
}
