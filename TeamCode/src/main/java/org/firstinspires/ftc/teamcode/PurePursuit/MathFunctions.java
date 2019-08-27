package org.firstinspires.ftc.teamcode.PurePursuit;

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
