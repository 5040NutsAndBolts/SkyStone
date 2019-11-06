package org.firstinspires.ftc.teamcode.competition.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class TowerArm {

    private Hardware robot;

    private int towerArmHeight = 1000;

    public TowerArm(Hardware hwMap) { robot = hwMap; }

    public void raiseLower(Hardware.TowerArmPos height){
        if(height == Hardware.TowerArmPos.RAISE) {
            robot.towerArmMotor.setTargetPosition(towerArmHeight);
            robot.towerArmMotor.setPower(1);
        }
        else if(height == Hardware.TowerArmPos.LOWER) {
            robot.towerArmMotor.setTargetPosition(0);
            robot.towerArmMotor.setPower(-1);
        }
        else if(height == Hardware.TowerArmPos.STOP){
            robot.towerArmMotor.setTargetPosition(robot.towerArmMotor.getCurrentPosition());
            robot.towerArmMotor.setPower(0);
        }
    }

    public void openClose(Hardware.ClawPos clawPos) {

        // Uses VEX 393 Servo Motors (360 degree servo)
        // setPosition(0) --> Turns counter clockwise
        // setPosition(1) --> Turns clockwise
        // setPosition(.5) --> Stops

        if (clawPos == Hardware.ClawPos.OPEN) {
            // Left claw
            robot.leftClaw.setPower(0);
            // Right claw
            robot.rightClaw.setPower(1);
        }
        else if (clawPos == Hardware.ClawPos.CLOSE) {
            // Left claw
            robot.leftClaw.setPower(1);
            // Right claw
            robot.rightClaw.setPower(0);
        }
        else if (clawPos == Hardware.ClawPos.STOP){
            robot.leftClaw.setPower(0.5);
            robot.rightClaw.setPower(0.5);
        }
    }

}
