package org.firstinspires.ftc.teamcode.competition.hardware;

public class LiftMech {

    private Hardware robot;
    public boolean clawOpen = true;
    private boolean clawExtended = true;

    public LiftMech(Hardware robot) { this.robot = robot; }

    /**
     * Opens or closes the claw
     */
    public void openClose() {
        clawOpen = !clawOpen;

        if (clawOpen)
            robot.claw.setPosition(1);
        else
            robot.claw.setPosition(.1);
    }

    public void openClaw() {
        robot.claw.setPosition(.1);
    }

    public void closeClaw() { robot.claw.setPosition(1); }

    /**
     * Extends or retracts the claw
     */
    public void extendRetract() {
        clawExtended = !clawExtended;

        if (clawExtended)
            robot.clawExtension.setPosition(0.5);
        else
            robot.clawExtension.setPosition(0);
    }

    public void extendClaw() {
        robot.clawExtension.setPosition(.5);
    }

    public void retractClaw() {
        robot.clawExtension.setPosition(0);
    }

    /**
     * Raises or lowers the lift mechanism with a specific power
     *
     * @param power Motor power that will be given to the lift mechanism
     */
    public void raiseLower(double power) {
        robot.liftMotor1.setPower(power);
        robot.liftMotor2.setPower(power);
    }
}
