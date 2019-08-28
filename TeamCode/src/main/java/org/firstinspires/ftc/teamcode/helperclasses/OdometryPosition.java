package org.firstinspires.ftc.teamcode.helperclasses;
import org.firstinspires.ftc.teamcode.competition.Hardware;
import org.firstinspires.ftc.teamcode.competition.MecanumDrive;

public class OdometryPosition extends Thread
{

    public static double worldXPosition=0;
    public static double worldYPosition=0;
    public static double worldAngle_rad=0;

    static Hardware robot = new Hardware();
    MecanumDrive drive = new MecanumDrive(robot);
    public void run()
    {

        robot.updatePosition();
        worldXPosition=robot.x;
        worldYPosition=robot.y;
        worldAngle_rad=robot.theta;

    }

}
