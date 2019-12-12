package org.firstinspires.ftc.teamcode.competition.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

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
    public double orientedAdjust = Math.toRadians(90);

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

    public void orientedDrive(double forward, double sideways, double rotation, boolean reset) {

        double P = Math.hypot(sideways, forward);
        robot.bulkData = robot.expansionHub.getBulkInputData();
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double robotAngle = Math.atan2(forward, -sideways);

        if(reset) {
            orientedAdjust = angles.firstAngle;
        }

        double v5 = P * Math.sin(robotAngle - angles.firstAngle + orientedAdjust) + P * Math.cos(robotAngle - angles.firstAngle + orientedAdjust) - rotation;
        double v6 = P * Math.sin(robotAngle - angles.firstAngle + orientedAdjust) - P * Math.cos(robotAngle - angles.firstAngle + orientedAdjust) + rotation;
        double v7 = P * Math.sin(robotAngle - angles.firstAngle + orientedAdjust) - P * Math.cos(robotAngle - angles.firstAngle + orientedAdjust) - rotation;
        double v8 = P * Math.sin(robotAngle - angles.firstAngle + orientedAdjust) + P * Math.cos(robotAngle - angles.firstAngle + orientedAdjust) + rotation;

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
}