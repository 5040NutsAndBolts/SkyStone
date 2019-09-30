package org.firstinspires.ftc.teamcode.helperclasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID
{

    private double distanceFromGoal;
    private double lastDistanceFromGoal;
    private double integral;
    private double p;
    private double i;
    private double d;

    private double pComponent;
    private double iComponent;
    private double dComponent;

    private ElapsedTime e = new ElapsedTime();
    private double elapsedTime;
    public PID(double goal, double current, double p, double i, double d)
    {

        this.p=p;
        this.i=i;
        this.d=d;
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
        pComponent=p();
        iComponent=i();
        dComponent=d();
        e.reset();
        lastDistanceFromGoal=distanceFromGoal;

    }

    private void setDistanceFromGoal(double d)
    {

        distanceFromGoal=d;

    }

    public double p()
        {return distanceFromGoal*d;}

    public double i()
        {return integral*i; }

    public double d()
        {return (distanceFromGoal-lastDistanceFromGoal)/elapsedTime;}

    /**
     *@return Returns the output of the PID controller
     */
    public double getPID()
        {return pComponent+iComponent+dComponent;}

}
