package org.firstinspires.ftc.teamcode.competition.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.competition.helperclasses.LineSegment;
import org.firstinspires.ftc.teamcode.competition.helperclasses.MathFunctions;
import org.opencv.core.Point;

import static java.lang.Math.abs;

/**
 * Class is for the mecanum drive code
 */
public class MecanumDrive {

    private Hardware robot;
    public double orientedAdjust = Math.toRadians(90);

    /**
     * sets up the hardware refernce so you don't have to pass it as a parameter and sets the adjust
     * @param r The hardware reference from the code
     */
    public MecanumDrive(Hardware r) { robot = r; }

    /**
     * Drives the robot with the front being a specific direction of the robot
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
     * Drives the robot with the front being any section of the robot facing an angle
     * @param forward  The forward value input (left stick y)
     * @param sideways The sideways value input (left stick x)
     * @param rotation The rotation value input (right stick x)
     * @param reset Whether or not to reset the angle the robot is oriented to
     */
    public void orientedDrive(double forward, double sideways, double rotation, boolean reset) {

        double P = Math.hypot(sideways, forward);
        double robotAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        double newRobotAngle = Math.atan2(forward, -sideways);

        if(reset) {
            orientedAdjust = robotAngle;
        }

        double v5 = P * Math.sin(newRobotAngle - robotAngle + orientedAdjust) + P * Math.cos(newRobotAngle - robotAngle + orientedAdjust) - rotation;
        double v6 = P * Math.sin(newRobotAngle - robotAngle + orientedAdjust) - P * Math.cos(newRobotAngle - robotAngle + orientedAdjust) + rotation;
        double v7 = P * Math.sin(newRobotAngle - robotAngle + orientedAdjust) - P * Math.cos(newRobotAngle - robotAngle + orientedAdjust) - rotation;
        double v8 = P * Math.sin(newRobotAngle - robotAngle + orientedAdjust) + P * Math.cos(newRobotAngle - robotAngle + orientedAdjust) + rotation;

        robot.leftFront.setPower(v5);
        robot.rightFront.setPower(v6);
        robot.leftRear.setPower(v7);
        robot.rightRear.setPower(v8);
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

    public void goToPosition(Point goalPoint)
    {

        double distanceToTarget = Math.hypot(goalPoint.x-robot.x,goalPoint.y-robot.y);
        double absoluteAngleToTarget = Math.atan2(goalPoint.y-robot.y,goalPoint.x-robot.x);
        double relativeAngleToPoint = absoluteAngleToTarget - (MathFunctions.angleWrap(robot.theta-Math.PI/2));
        double relativeXToPoint = Math.cos(relativeAngleToPoint)*distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint)*distanceToTarget;
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint)+Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeYToPoint)+Math.abs(relativeXToPoint));
        double movementTurn;
        relativeAngleToPoint = absoluteAngleToTarget - (MathFunctions.angleWrap(robot.theta));


        movementTurn = (relativeAngleToPoint) / Math.sqrt(Math.abs(relativeAngleToPoint )) ;


        drive(-movementYPower,movementXPower,movementTurn/5);

    }

}