package org.firstinspires.ftc.teamcode.competition.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.PID;
import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;

@Config
public class Carriage {

    private Hardware robot;
    public static double
            extendedPosition1 = -4700,
            extendedPosition2 = -9100,
            retractedPosition = 0,
            goalPosition = 0;

    public static double
        P = 0.002,
        I = 0.000001,
        D = 0.00009;

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
    public double elapTime=0;
    private void initThread() {
        Thread carriageThread = new Thread() {
            PID carriagePID = new PID(goalPosition - robot.intakeRight.getCurrentPosition(), P, I, D);
            CarriagePosition lastState = carriageState;

            @Override
            public void run() {
                double lastTime=0;
                ElapsedTime time = new ElapsedTime();
                time.startTime();
                // Essentially the same as while(opModeIsActive())
                while(!this.isInterrupted()) {
                    // Don't use PID if we are manually controlling it
                    if (carriageState != CarriagePosition.Manual) {
                        // If the state has changed since the last run, update the PID accordingly
                        if (lastState != carriageState)
                            carriagePID = new PID(goalPosition - robot.intakeRight.getCurrentPosition(), P, I, D);

                        // If the motor isn't within 25 ticks of the goal position, move the claw
                        if (!(robot.intakeRight.getCurrentPosition()+25 > goalPosition &&
                                robot.intakeRight.getCurrentPosition()-25 < goalPosition)) {
                            setPower(carriagePID.getPID());

                            carriagePID.update(goalPosition - robot.intakeRight.getCurrentPosition());
                        }
                        else
                            setPower(0);
                    }

                    lastState = carriageState;
                    elapTime=time.milliseconds()-lastTime;
                    lastTime=time.milliseconds();
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
        if (carriageState == CarriagePosition.Manual) {
            power = HelperMethods.clamp(-.85, power, .85);
            robot.clawExtension1.setPower(power);
            robot.clawExtension2.setPower(power);
        }
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
                extendedPosition1:
                extendedPosition2;
    }

    /**
     * Sets the carriage variables to retracted position
     */
    public void retract() {
        carriageState = CarriagePosition.Retracted;
        goalPosition = retractedPosition;
    }

    /**
     * Opens the claw
     */
    public void openClaw() { robot.claw.setPosition(.5); }

    /**
     * Closes the claw
     */
    public void closeClaw() { robot.claw.setPosition(0); }

    /**
     * Resets the carriage encoder
     */
    public void reset() {
        robot.intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
