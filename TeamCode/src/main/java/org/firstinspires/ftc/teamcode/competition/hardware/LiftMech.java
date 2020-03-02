package org.firstinspires.ftc.teamcode.competition.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.helperclasses.PID;
import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;

@Config
public class LiftMech {

    private Hardware robot;
    private int stackLevel = 0;
    public static double
        P = 0,
        I = 0,
        D = 0;
    public static int
            height0 = 0,
            height1 = 0,
            height2 = 0,
            height3 = 0,
            height4 = 0,
            height5 = 0,
            height6 = 0,
            height7 = 0,
            height8 = 0,
            height9 = 0,
            height10 = 0,
            height11 = 0,
            height12 = 0,
            height13 = 0,
            height14 = 0,
            height15 = 0;
    private int[] goalPosition = new int[16];
    public enum LiftState {
        Manual,
        Holding,
        Moving
    }
    public LiftState currentState = LiftState.Holding;

    public LiftMech(Hardware robot) {
        this.robot = robot;
        initThread();
    }

    /**
     * Creates a new thread for the lift mechanism
     */
    private void initThread() {
        Thread liftThread = new Thread() {
            @Override
            public void run() {
                PID liftPID = new PID(goalPosition[stackLevel] - robot.intakeLeft.getCurrentPosition(), P, I, D);
                LiftState lastState = LiftState.Holding;

                // Essentially the same as while(opModeIsActive())
                while(!this.isInterrupted()) {
                    // If not running manual mode
                    if (currentState != LiftState.Manual) {
                        // If state == holding or the lift is within 3% of its goal position, then hold its current position
                        if (currentState == LiftState.Holding ||
                                (robot.intakeLeft.getCurrentPosition()+100 > goalPosition[stackLevel] &&
                                        robot.intakeLeft.getCurrentPosition()-100 < goalPosition[stackLevel])) {
                            currentState = LiftState.Holding;
                            setPower(0);
                        }
                        else { // Otherwise, state == Moving
                            if (lastState != currentState) // If state just became moving, reset the PID
                                liftPID = new PID(goalPosition[stackLevel] - robot.intakeLeft.getCurrentPosition(), P, I, D);

                            setPower(liftPID.getPID());

                            liftPID.update(goalPosition[stackLevel] - robot.intakeLeft.getCurrentPosition());
                        }
                    }

                    lastState = currentState;
                }
            }
        };

        ThreadPool.pool.submit(liftThread);
    }



    /**
     * Manual control over the lift
     * @param power Motor power that will be given to the lift mechanism
     */
    public void manual(double power) {
        if (currentState == LiftState.Manual) {
            robot.liftMotor1.setPower(power);
            robot.liftMotor2.setPower(power);
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
        stackLevel = level;
        currentState = LiftState.Moving;
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
