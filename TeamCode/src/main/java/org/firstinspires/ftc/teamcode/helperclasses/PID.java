package org.firstinspires.ftc.teamcode.helperclasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID
{

    private double distanceFromGoal;
    private double lastdistanceFromGoal;
    private double integral;
    private double p;
    private double i;
    private double d;

    private double pComponent;
    private double iComponent;
    private double dComponent;

    private ElapsedTime e = new ElapsedTime();
    private double elapsedTime;
    public PID(double distanceFromGoal, double p, double i, double d)
    {

        this.p=p;
        this.i=i;
        this.d=d;
        this.distanceFromGoal=distanceFromGoal;
        e.startTime();

    }

    public void update(double goal,double position)
    {

        setDistancefromGoal(goal-position);
        elapsedTime=e.time();
        integral+=distanceFromGoal*elapsedTime;
        pComponent=p();
        iComponent=i();
        dComponent=d();
        e.reset();
        lastdistanceFromGoal=distanceFromGoal;

    }

    public void setDistancefromGoal(double d)
    {

        distanceFromGoal=d;

    }

    public double p()
        {return distanceFromGoal*d;}

    public double i()
        {return integral*i; }

    public double d()
        {return (distanceFromGoal-lastdistanceFromGoal)/elapsedTime;}

    public double getPID()
        {return pComponent+iComponent+dComponent;}

}
