package org.firstinspires.ftc.teamcode.competition.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.PID;
import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;

public class Carriage {

    private Hardware robot;
    private double
            extendedPosition1 = 0,
            extendedPosition2 = 0,
            retractedPosition = 0,
            goalPosition = 0;

    public enum CarriagePosition {
        Manual,
        Extended1,
        Extended2,
        Retracted
    }
    public Carriage.CarriagePosition carriageState = Carriage.CarriagePosition.Manual;

    public Carriage(Hardware robot) {
        this.robot = robot;
        initThread();
    }

    /**
     * Create a new thread for the carriage mechanism
     */
    private void initThread() {
        Thread carriageThread = new Thread() {
            PID carriagePID = new PID(goalPosition - robot.liftMotor1.getCurrentPosition(), .4, 3, .2);
            CarriagePosition lastState = carriageState;

            @Override
            public void run() {
                // Essentially the same as while(opModeIsActive())
                while(this.isAlive()) {
                    // Don't use PID if we are manually controlling it
                    if (carriageState != CarriagePosition.Manual) {
                        // If the state has changed since the last run, update the PID accordingly
                        if (lastState != carriageState)
                            carriagePID = new PID(goalPosition - robot.liftMotor1.getCurrentPosition(), .4, 3, .2);

                        // If the motor isn't within 3% of the goal position, move the claw
                        if (!HelperMethods.inThreshold(robot.liftMotor1.getCurrentPosition(), goalPosition, 3)) {
                            manual(carriagePID.getPID());

                            carriagePID.update(goalPosition - robot.liftMotor1.getCurrentPosition());
                        }
                    }

                    lastState = carriageState;
                }
            }
        };

        ThreadPool.pool.submit(carriageThread);
    }

    /**
     * Manual power control over the carriage
     * @param power Power to be put into the carriage
     */
    public void manual(double power) {
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
                extendedPosition2 :
                extendedPosition1;
    }

    /**
     * Sets the carriage variables to retracted position
     */
    public void retract() {
        carriageState = CarriagePosition.Retracted;
        goalPosition = retractedPosition;
    }

    /**
     * Resets the carriage encoder
     */
    public void reset() {
        robot.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}