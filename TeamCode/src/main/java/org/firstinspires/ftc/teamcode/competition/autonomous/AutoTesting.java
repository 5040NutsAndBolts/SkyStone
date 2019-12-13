package org.firstinspires.ftc.teamcode.competition.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group="Auto",name = "Auto testing")
public class AutoTesting extends AutoMethods {

    private boolean startingOnBlue = true;
    private boolean moveOnX = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // Lets us move the robot during auto (disables brakes)
        drive.softBrakeMotors();

        robot.resetOdometry(135,9,0);

        while (!isStarted()) {
            telemetry.addData("Robot facing", robot.facing);
            telemetry.addLine();
            telemetry.addLine("If resetting pos, please orient it w/ the front against the wall");
            telemetry.addLine();
            if (moveOnX)
                telemetry.addLine("Moving on X-Axis");
            else
                telemetry.addLine("Moving on Y-Axis");
            telemetry.addLine();
            if (startingOnBlue)
                telemetry.addLine("We are blue alliance");
            else
                telemetry.addLine("We are red alliance");
            updateOdometryTeleop();

            if (gamepad1.dpad_left) {
                startingOnBlue = true;
                robot.resetOdometry(135,9,0);
            }
            if (gamepad1.dpad_right) {
                startingOnBlue = false;
                robot.resetOdometry(9,9,Math.PI);
            }
            if (gamepad1.y)
                moveOnX = false;
            if (gamepad1.x)
                moveOnX = true;

        }

        if (moveOnX)
            xAxisMoveTo(21,3);
        else
            yAxisMoveTo(21,3);

        while(opModeIsActive()) {
            telemetry.addLine("Auto has ended");
            telemetry.addLine("==========");
            updateOdometryTeleop();
        }

    }
}
