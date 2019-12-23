package org.firstinspires.ftc.teamcode.competition.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.competition.helperclasses.HelperMethods;
import static org.firstinspires.ftc.teamcode.competition.helperclasses.HelperMethods.getQuadrant;

@Autonomous(group="Auto",name = "Auto testing")
public class AutoTesting extends AutoMethods {

    private boolean startingOnBlue = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // Lets us move the robot during auto (disables brakes)
        drive.softBrakeMotors();

        robot.resetOdometry(135,9,0);

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
                robot.resetOdometry(135,9,0);
            }
            if (gamepad1.dpad_right) {
                startingOnBlue = false;
                robot.resetOdometry(9,9,Math.PI);
            }

        }



        endAuto();

    }
}

