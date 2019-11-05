package org.firstinspires.ftc.teamcode.competition.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

/**
 * Class is for the mecanum drive code
 */
public class MecanumDrive {

    private Hardware robot;

    public double adjust = 0;

    /**
     * sets up the hardware refernce so you don't have to pass it as a parameter and sets the adjust
     * @param r The hardware reference from the code
     */
    public MecanumDrive(Hardware r) { robot = r; }

    /**
     * this method is for driving the mecanum with the three inputs
     *
     * @param forward  The forward value input (left stick y)
     * @param sideways The sideways value input (left stick x)
     * @param rotation The rotation value input (right stick x)
     */
    public void drive(double forward, double sideways, double rotation) {
        //adds all the inputs together to get the number to scale it by
        double scale = abs(rotation) + abs(forward) + abs(sideways);

        //scales the inputs when needed
        if(scale > 1) {
            forward /= scale;
            rotation /= scale;
            sideways /= scale;
        }
        //setting the motor powers to move
        robot.leftFront.setPower(forward-rotation-sideways);
        robot.leftRear.setPower(forward-rotation+sideways);
        robot.rightFront.setPower(forward+rotation+sideways);
        robot.rightRear.setPower(forward+rotation-sideways);
        //Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
        //Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe
    }

    /**
     * Field oriented drive for robot
     * Sets different sides to be the front of the robot
     *
     * @param forward  The forward value input
     * @param sideways The sideways value input
     * @param rotation The rotation value input
     * @param reset Resets orientation to whichever direction the driver is facing
     */
    public void orientedDrive(double forward, double sideways, double rotation, boolean reset) {

        double P = Math.hypot(sideways, forward);
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double robotAngle = Math.atan2(forward, -sideways);

        if (reset) adjust = angles.firstAngle;

        double v5 = P * Math.sin(robotAngle - angles.firstAngle + adjust) + P * Math.cos(robotAngle - angles.firstAngle + adjust) - rotation;
        double v6 = P * Math.sin(robotAngle - angles.firstAngle + adjust) - P * Math.cos(robotAngle - angles.firstAngle + adjust) + rotation;
        double v7 = P * Math.sin(robotAngle - angles.firstAngle + adjust) - P * Math.cos(robotAngle - angles.firstAngle + adjust) - rotation;
        double v8 = P * Math.sin(robotAngle - angles.firstAngle + adjust) + P * Math.cos(robotAngle - angles.firstAngle + adjust) + rotation;

        powerSet(v5, v6, v7, v8);
    }

    /**
     * Sets single power to all motors
     * @param power The power the robot is set to 0-1
     */
    public void powerSet(double power) {
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftRear.setPower(power);
        robot.rightRear.setPower(power);
    }

    /**
     * Sets specific powers of motors
     * @param v1 Left front power value
     * @param v2 Right front power value
     * @param v3 Left rear power value
     * @param v4 Right rear power value
     */
    public void powerSet(double v1, double v2, double v3, double v4) {
        robot.leftFront.setPower(v1);
        robot.rightFront.setPower(v2);
        robot.leftRear.setPower(v3);
        robot.rightRear.setPower(v4);
    }

    /**
     * Brakes the motors so robot can't move
     */
    public void brakeMotors() { powerSet(0); }
}
