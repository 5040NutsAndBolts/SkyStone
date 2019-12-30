package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.PurePursuit.CheckPoint;
import org.firstinspires.ftc.teamcode.PurePursuit.WayPoint;
import org.firstinspires.ftc.teamcode.competition.visiontesting.SkystonePipeline;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.IntakeMech;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.competition.helperclasses.ThreadPool.pool;
import static org.firstinspires.ftc.teamcode.competition.visiontesting.SkystonePipeline.screenPosition;

@Autonomous(name = "OpenCVSkyStoneAutoPurePursuit", group = "Auto")
public class OpenCVSkyStoneAutoPurePursuit extends LinearOpMode {
    Hardware robot = new Hardware();
    MecanumDrive drive = new MecanumDrive(robot);
    IntakeMech intake = new IntakeMech(robot);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        int xPosition = screenPosition.x;
        PurePursuit purePursuit = new PurePursuit(robot);
        int blockPosition = (xPosition < 12) ? 1 : (((xPosition < 30) && (xPosition > 12)) ? 2 : 3);
        if(blockPosition == 1){
            CheckPoint c1 = new CheckPoint(0, 0, Math.PI/4, robot);
            ArrayList<WayPoint> p1 = new ArrayList();
            p1.add(new WayPoint(5, 36, 0));
            p1.add(new WayPoint(40,40,Math.PI/4));
            p1.add(new WayPoint(49,31,Math.PI/4));
            purePursuit.initPath(p1, .4, .005, .55);
            pool.submit(c1);

        }
        if(blockPosition == 2){




        }
        if(blockPosition == 3){



        }

    }
}
