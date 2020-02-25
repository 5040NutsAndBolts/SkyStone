package org.firstinspires.ftc.teamcode.competition.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.PID;
import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;

public class LiftMech {

    private Hardware robot;
    private int stackLevel = 0;
    private int[] goalPosition = {
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
    };
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
                PID liftPID = new PID(goalPosition[stackLevel] - robot.intakeLeft.getCurrentPosition(), 0, 0, 0);
                LiftState lastState = LiftState.Holding;

                // Essentially the same as while(opModeIsActive())
                while(this.isAlive()) {
                    // If not running manual mode
                    if (currentState != LiftState.Manual) {
                        // If state == holding or the lift is within 3% of its goal position, then hold its current position
                        if (currentState == LiftState.Holding ||
                                HelperMethods.inThreshold(robot.intakeLeft.getCurrentPosition(), goalPosition[stackLevel], 3)) {
                            currentState = LiftState.Holding;
                            manual(0);
                        }
                        else { // Otherwise, state == Moving
                            if (lastState != currentState) // If state just became moving, reset the PID
                                liftPID = new PID(goalPosition[stackLevel] - robot.intakeLeft.getCurrentPosition(), 0, 0, 0);

                            manual(liftPID.getPID());

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
     * Opens the claw
     */
    public void openClaw() { robot.claw.setPosition(.1); }

    /**
     * Closes the claw
     */
    public void closeClaw() { robot.claw.setPosition(1); }

    /**
     * Manual control over the lift
     * @param power Motor power that will be given to the lift mechanism
     */
    public void manual(double power) {
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
}
