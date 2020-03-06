package org.firstinspires.ftc.teamcode.competition.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helperclasses.PID;
import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;

@Config
public class LiftMech {

    private Hardware robot;
    private int stackLevel = 0;
    public static double liftHoldPower = -.05;
    public static double
            P = .001,
            I = 0,
            D = 0;
    public static int
            height0 = 0,
            height1 = 0,
            height2 = -4800,
            height3 = -9600,
            height4 = -14400,
            height5 = -19200,
            height6 = -24000,
            height7 = -28800,
            height8 = -33600,
            height9 = -38400,
            height10 = -43200,
            height11 = -48000,
            height12 = -52800,
            height13 = -57600,
            height14 = -62400,
            height15 = -67200;
    private int[] goalPosition = new int[16];
    public double speed = 0;

    public enum LiftState {
        Manual,
        Holding,
        Moving
    }

    public PID pid;
    public LiftState currentState = LiftState.Holding;

    public LiftMech(Hardware robot) {
        this.robot = robot;
        pid = new PID(0, P, I, D);
    }

    /**
     * Runs the lift based on the
     * @param g Gamepad to get left stick value from
     */
    public void run(Gamepad g) {
        if (g.left_stick_y != 0) {
            currentState = LiftState.Manual;
            setPower(g.left_stick_y);
        } else if (currentState == LiftState.Moving) {
            if (pid == null)
                pid = new PID(goalPosition[stackLevel] - robot.intakeLeft.getCurrentPosition(), P, I, D);
            pid.update(goalPosition[stackLevel] - robot.intakeLeft.getCurrentPosition());
            setPower(pid.getPID());
            if (Math.abs(goalPosition[stackLevel] - robot.intakeLeft.getCurrentPosition()) < 30)
                currentState = LiftState.Holding;
        } else {
            currentState = LiftState.Holding;
            setPower(liftHoldPower);
        }
    }

    public void setPower(double power) {
        robot.liftMotor1.setPower(power);
        robot.liftMotor2.setPower(power);
    }

    /**
     * Raises the lift of the robot to a specified level
     */
    public void moveToLevel(int level) {
        updateHeights();
        stackLevel = level;
        currentState = LiftState.Moving;
        pid = null;
    }

    /**
     * Resets the carriage encoder
     */
    public void reset() {
        robot.intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateHeights() {
        goalPosition[0] = height0;
        goalPosition[1] = height1;
        goalPosition[2] = height2;
        goalPosition[3] = height3;
        goalPosition[4] = height4;
        goalPosition[5] = height5;
        goalPosition[6] = height6;
        goalPosition[7] = height7;
        goalPosition[8] = height8;
        goalPosition[9] = height9;
        goalPosition[10] = height10;
        goalPosition[11] = height11;
        goalPosition[12] = height12;
        goalPosition[13] = height13;
        goalPosition[14] = height14;
        goalPosition[15] = height15;
    }
}
