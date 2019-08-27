package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.teamcode.helperclasses.OdometryPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.helperclasses.OdometryPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.helperclasses.OdometryPosition.worldYPosition;

public class RobotVars extends Thread
{

    //location in centimeters
    public static double x = worldXPosition;
    public static double y = worldYPosition;
    //rotation in radians
    public static double r = worldAngle_rad;
    //velocity in meters per second
    public static double velocity = 0;
    //angular velocity in rotations per second
    public static double angularVelocity = 0;
    //acceleration in meters per second^2
    public static double acceleration = 0;
    //angular acceleration in rotations per second^2
    public static double angularAcceleration=0;

    private double lastVelocity=0;

    private double lastAngularVelocity=0;

    private long lastUpdateTime = 0;

    public boolean stop=false;

    public void run()
    {

        while(!stop)
        {
            long currentTimeMillis = System.currentTimeMillis();
            double elapsedTime = (currentTimeMillis - lastUpdateTime) / 1000.0;
            lastUpdateTime = currentTimeMillis;
            velocity = Math.hypot(worldXPosition - x, worldYPosition - y) / elapsedTime / 1000;
            acceleration = (velocity - lastVelocity) / elapsedTime;
            angularVelocity = (worldAngle_rad - r) / Math.PI / 2 / elapsedTime;
            angularAcceleration = (angularVelocity - lastAngularVelocity) / elapsedTime;
            lastVelocity = velocity;
            x = worldXPosition;
            y = worldYPosition;
            r = worldAngle_rad;
        }

    }

}
