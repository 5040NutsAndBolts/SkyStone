package org.firstinspires.ftc.teamcode.helperclasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID
{

    private double distanceFromGoal;
    private double lastDistanceFromGoal=0;
    private double integral;
    double derivative;
    private double kP;
    private double kI;
    private double kD;

    private double pComponent;
    private double iComponent;
    private double dComponent;

    private ElapsedTime e = new ElapsedTime();
    private double elapsedTime;

    /**
     *
     * @param goal The position the controller is headed towards.
     * @param current The position the controller in at.
     * @param p The proportional gain.
     * @param i The integral gain.
     * @param d The derivative gain.
     */
    public PID(double goal, double current, double p, double i, double d)
    {

        this.kP=p;
        this.kI=i;
        this.kD=d;
        distanceFromGoal=goal-current;
        e.startTime();

    }

    /**
     * Updates the PID controller, this should be called every before the controller is used
     * @param goal The position the PID controller heads to
     * @param position The current position of the system
     */
    public void update(double goal,double position)
    {

        setDistanceFromGoal(goal-position);
        elapsedTime=e.time();
        integral+=distanceFromGoal*elapsedTime;
        derivative=(distanceFromGoal-lastDistanceFromGoal)/elapsedTime;
        pComponent=p();
        iComponent=i();
        dComponent=d();
        e.reset();
        lastDistanceFromGoal=distanceFromGoal;

    }

    //sets the value of the distance from goal for the controller
    private void setDistanceFromGoal(double d)
    {

        distanceFromGoal=d;

    }

    //calculates the proportional component
    public double p()
        {return distanceFromGoal*kP;}

    //calculates the integral component
    public double i()
        {return integral*kI; }

    //calculates the derivative component
    public double d()
        {return derivative*kD;}

    /**
     *@return Returns the output of the PID controller
     */
    public double getPID()
        {return pComponent+iComponent+dComponent;}
    public void resetPid()
    {

        lastDistanceFromGoal=0;
        integral=0;

    }

}
