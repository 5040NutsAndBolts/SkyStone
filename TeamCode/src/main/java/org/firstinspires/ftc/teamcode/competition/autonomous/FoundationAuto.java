package org.firstinspires.ftc.teamcode.competition.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group="Auto",name = "Foundation Auto")
public class FoundationAuto extends AutoMethods {

    private boolean startingOnBlue = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // Lets us move the robot during auto (disables brakes)
        drive.softBrakeMotors();

        robot.resetOdometry(111,111,0);

        while (!isStarted()) {
            telemetry.addLine("If resetting pos, please orient it w/ the front against the wall");
            telemetry.addLine("D-Pad = Alliance Selection");
            telemetry.addLine();
            if (startingOnBlue)
                telemetry.addLine("We are blue alliance");
            else
                telemetry.addLine("We are red alliance");
            updateOdometryTeleop();

            if (gamepad1.dpad_left) {
                startingOnBlue = true;
                robot.resetOdometry(111,111,0);
            }
            if (gamepad1.dpad_right) {
                startingOnBlue = false;
                robot.resetOdometry(9,111, Math.PI);
            }
        }



        displayEndAuto();

    }
}

