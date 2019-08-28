package org.firstinspires.ftc.teamcode.competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.competition.Hardware;
import org.firstinspires.ftc.teamcode.competition.MecanumDrive;
import org.firstinspires.ftc.teamcode.helperclasses.WayPoint;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.helperclasses.OdometryPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.helperclasses.OdometryPosition.worldYPosition;
@Autonomous(name = "purepursuit", group = "Auto")
public class PurePursuitTest extends LinearOpMode
{


    ArrayList<org.firstinspires.ftc.teamcode.PurePursuit.WayPoint> p;
    PurePursuit purePursuit = new PurePursuit();
    Hardware robot = new Hardware();
    MecanumDrive drive = new MecanumDrive(robot);
    @Override
    public void runOpMode() throws InterruptedException
    {

        p.add(0, new org.firstinspires.ftc.teamcode.PurePursuit.WayPoint(20,20,0,1,1));
        while(!(worldXPosition>p.get(p.size()-2).x-3&&worldXPosition<p.get(p.size()-2).x+3&&worldYPosition>p.get(p.size()-2).y-3&&worldYPosition<p.get(p.size()-2).y+3))
        {

            purePursuit.followPath(p,40);

        }
        drive.brakeMotors();


    }
}
