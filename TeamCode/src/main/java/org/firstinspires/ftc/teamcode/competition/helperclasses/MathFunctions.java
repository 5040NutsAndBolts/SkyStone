package org.firstinspires.ftc.teamcode.competition.helperclasses;

public class MathFunctions
{

    public static double angleWrap(double angle)
    {

        while(angle>=2*Math.PI)
            {angle-=2*Math.PI;}

        while(angle<0)
            {angle+=2*Math.PI;}

        return angle;

    }

}
