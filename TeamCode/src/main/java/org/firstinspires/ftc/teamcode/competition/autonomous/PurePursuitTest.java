package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PurePursuit.CheckPoint;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.PurePursuit.WayPoint;

import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.IntakeMech;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.competition.helperclasses.ThreadPool.pool;


@Autonomous(name = "PP", group = "Auto")
public class PurePursuitTest extends LinearOpMode {
    private Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        PurePursuit purePursuit = new PurePursuit(robot);

        ArrayList<WayPoint> p = new ArrayList();
        p.add(new WayPoint(12, 0, 0));
        p.add(new WayPoint(12, -24, 0));
        p.add(new WayPoint(0, -24, 0));

        waitForStart();

        purePursuit.initPath(p);
        while (opModeIsActive()) {
            robot.updatePositionRoadRunner();

            purePursuit.followPath(p, 5, 1);

            telemetry.addData("Last Goal X", purePursuit.lastGoal[0]);
            telemetry.addData("Last Goal Y", purePursuit.lastGoal[1]);
            telemetry.addData("pid", purePursuit.pos.getPID());
            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("heading", robot.theta);
            telemetry.update();
        }
    }
}
