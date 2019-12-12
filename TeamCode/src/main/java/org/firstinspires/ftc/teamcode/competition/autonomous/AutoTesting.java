package org.firstinspires.ftc.teamcode.competition.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group="Auto",name = "Auto testing")
public class AutoTesting extends AutoMethods {

    private boolean startingOnBlue = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // Lets us move the robot during auto (disables brakes)
        drive.softBrakeMotors();

        robot.resetOdometry(0,0,-Math.PI);

        while (!isStarted()) {
            telemetry.addLine("If resetting robot, please orient it with the front against the wall");
            telemetry.addLine("==========");
            if (startingOnBlue)
                telemetry.addLine("We are blue alliance");
            else
                telemetry.addLine("We are red alliance");
            updateOdometryTeleop();

            if (gamepad1.x) {
                startingOnBlue = true;
                robot.resetOdometry(126,0,0);
            }
            if (gamepad1.b) {
                startingOnBlue = false;
                robot.resetOdometry(0,0,-Math.PI);
            }

        }

        xAxisMoveTo(12,10);

        while(opModeIsActive()) {
            telemetry.addLine("Auto has ended");
            telemetry.addLine("==========");
            updateOdometryTeleop();
        }

    }
}
