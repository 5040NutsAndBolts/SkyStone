package org.firstinspires.ftc.teamcode.competition.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.competition.helperclasses.HelperMethods;

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

    public void driveSideWays(double speed,double angle)
    {



        double rotation=0;

        if(!HelperMethods.inThreshhold(angle,robot.theta,.01))
        {

            rotation=(angle-robot.theta)/2;

        }
        double scale = abs(rotation)  + abs(speed);

        //scales the inputs when needed
        if(scale > 1) {
            rotation /= scale;
            speed /= scale;
        }
        robot.leftFront.setPower(rotation-speed);
        robot.leftRear.setPower(rotation-speed);
        robot.rightFront.setPower(-rotation+speed);
        robot.rightRear.setPower(-rotation-speed);

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
     * Sets power of all motors to a single value
     * @param power The power the robot is set to 0-1
     */
    public void powerSet(double power) {
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftRear.setPower(power);
        robot.rightRear.setPower(power);
    }

    /**
     * Sets individual powers of the motors
     * @param v Left front motor power
     * @param v1 Right front motor power
     * @param v2 Left rear motor power
     * @param v3 Right rear motor power
     */
    public void powerSet(double v, double v1, double v2, double v3) {
        robot.leftFront.setPower(v);
        robot.rightFront.setPower(v1);
        robot.leftRear.setPower(v2);
        robot.rightRear.setPower(v3);
    }

    /**
     * Brakes the motors so robot can't move
     */
    public void hardBrakeMotors() {
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        powerSet(0);
    }

    /**
     * Sets the zero power behavior to not brake the motors
     * Used during autonomous to make the robot easier to move
     */
    public void  softBrakeMotors() {
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        powerSet(0);
    }
}