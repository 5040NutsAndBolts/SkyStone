package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.competition.hardware.*;

@Autonomous(name = "Parking Only", group = "Auto")
public class ParkingOnlyAuto extends LinearOpMode {

    private enum StartLoc {
        Depot, Middle, Bridge;
        static final StartLoc[] VALUES = values();
        public StartLoc incrementSize() { return VALUES[ordinal()+1]; }
        public StartLoc decrementSize() { return VALUES[ordinal()-1]; }
    }

    private Hardware robot;
    private MecanumDrive driveTrain;
    private StartLoc startLoc = StartLoc.Bridge;

    private int waitTimeMS = 0;
    private boolean endTouchingWall = true;
    private boolean pressedDPad = false;

    @Override
    public void runOpMode() {
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);

        // Init loop
        while (!isStarted()&&!isStopRequested()) {
            telemetry.addData("Robot starting position", startLoc);
            telemetry.addData("Robot ends touching wall", endTouchingWall);
            telemetry.addData("Time before auto starts (sec.)", waitTimeMS/1000);
            telemetry.update();

            if (gamepad1.dpad_up && !pressedDPad)
                startLoc.incrementSize();

            if (gamepad1.dpad_down && !pressedDPad)
                startLoc.decrementSize();

            if (gamepad1.dpad_right && !pressedDPad)
                waitTimeMS += waitTimeMS < 20*1000 ? 100 : 0;

            if (gamepad1.dpad_left && !pressedDPad)
                waitTimeMS -= waitTimeMS > 0 ? 100 : 0;

            if (gamepad1.y)
                endTouchingWall = false;

            if (gamepad1.a)
                endTouchingWall = true;

            pressedDPad = gamepad1.dpad_up ||
                    gamepad1.dpad_down ||
                    gamepad1.dpad_left ||
                    gamepad1.dpad_right;
        }

        long endTime = System.currentTimeMillis() + waitTimeMS;
        while (System.currentTimeMillis() < endTime);

    }
}
