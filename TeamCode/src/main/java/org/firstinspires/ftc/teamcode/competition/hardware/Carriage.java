package org.firstinspires.ftc.teamcode.competition.hardware;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.PID;

@Config
public class Carriage {

    private Hardware robot;
    public static double
            extendedPosition1 = -5000,
            extendedPosition2 = -9100,
            retractedPosition = 0,
            goalPosition = 0;

    public static final double[] extensions1 = new double[]{
            -5000,
            -5000,
            -5000,
            -5000,
            -5000,
            -5000,
            -5000,
            -4500,
            -4300,
            -4100,
            -4000,
            -3800,
            -3700,
            -3700,
            -3600,
            -3230,
            -2930,
    };
    public static final double[] extensions2 = new double[]{
            -9100,
            -9100,
            -9100,
            -9100,
            -9100,
            -9100,
            -9100,
            -9100,
            -9100,
            -9100,
            -9100,
            -9100,
            -9100,
            -9100,
            -9100,
            -9100,
            -9100
    };

    public static double
            P = 0.0005,
            I = 0,
            D = 0.00005;

    public enum CarriagePosition {
        Manual,
        Extended1,
        Extended2,
        Retracted
    }

    public Carriage.CarriagePosition carriageState = Carriage.CarriagePosition.Manual;
    public boolean atPosition;

    PID pid;

    public Carriage(Hardware robot) {
        this.robot = robot;
        pid = new PID(0, P, I, D);
    }


    public void run(Gamepad g) {
        if (carriageState == CarriagePosition.Manual || g.right_stick_y != 0) {
            carriageState = CarriagePosition.Manual;
            setPower(g.right_stick_y/2);
        } else if (carriageState == CarriagePosition.Extended1) {
            if (pid == null)
                pid = new PID(extendedPosition1 - robot.intakeRight.getCurrentPosition(), P, I, D);
            pid.update(extendedPosition1 - robot.intakeRight.getCurrentPosition());
            setPower(pid.getPID());
            if (Math.abs(extendedPosition1 - robot.intakeRight.getCurrentPosition()) < 30)
                carriageState = CarriagePosition.Manual;
        } else if (carriageState == CarriagePosition.Extended2) {
            if (pid == null)
                pid = new PID(extendedPosition2 - robot.intakeRight.getCurrentPosition(), P, I, D);
            pid.update(extendedPosition2 - robot.intakeRight.getCurrentPosition());
            setPower(pid.getPID());
            if (Math.abs(extendedPosition2 - robot.intakeRight.getCurrentPosition()) < 30)
                carriageState = CarriagePosition.Manual;
        } else if (carriageState == CarriagePosition.Retracted) {
            if (pid == null)
                pid = new PID(retractedPosition - robot.intakeRight.getCurrentPosition(), P, I, D);
            pid.update(retractedPosition - robot.intakeRight.getCurrentPosition());
            setPower(pid.getPID());
            if (Math.abs(retractedPosition - robot.intakeRight.getCurrentPosition()) < 30)
                carriageState = CarriagePosition.Manual;
        }
    }

    public void setExtendedPosition(int stackHeight) {
        extendedPosition1 = extensions1[stackHeight];
        extendedPosition2 = extensions2[stackHeight];
    }

    private void setPower(double power) {
        power = HelperMethods.clamp(-.85, power, .85);
        robot.clawExtension1.setPower(power);
        robot.clawExtension2.setPower(power);
    }

    /**
     * Sets the carriage variables to either the first or second position
     */
    public void extend() {
        // Changing the carriageState
        carriageState = (carriageState == CarriagePosition.Extended1) ?
                CarriagePosition.Extended2 :
                CarriagePosition.Extended1;
        // Changing the goalPosition
        goalPosition = (carriageState == CarriagePosition.Extended1) ?
                extendedPosition1 :
                extendedPosition2;
        pid = null;
    }

    /**
     * Sets the carriage variables to retracted position
     */
    public void retract() {
        carriageState = CarriagePosition.Retracted;
        goalPosition = retractedPosition;
        pid = null;
    }

    /**
     * Opens the claw
     */
    public void openClaw() {
        robot.claw.setPosition(.5);
    }

    /**
     * Closes the claw
     */
    public void closeClaw() {
        robot.claw.setPosition(0);
    }

    /**
     * Resets the carriage encoder
     */
    public void reset() {
        robot.intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
