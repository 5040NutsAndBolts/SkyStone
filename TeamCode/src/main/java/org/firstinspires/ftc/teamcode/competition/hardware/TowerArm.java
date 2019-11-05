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
        // setPosition(0) --> Turns
        // setPosition(.5) --> Stops

        if (clawPos == Hardware.ClawPos.OPEN) {
            // Left claw
            robot.leftClaw.setDirection(Servo.Direction.REVERSE);
            robot.leftClaw.setPosition(0);
            // Right claw
            robot.rightClaw.setDirection(Servo.Direction.FORWARD);
            robot.rightClaw.setPosition(0);
        }
        else if (clawPos == Hardware.ClawPos.CLOSE) {
            // Left claw
            robot.leftClaw.setDirection(Servo.Direction.FORWARD);
            robot.leftClaw.setPosition(0);
            // Right claw
            robot.rightClaw.setDirection(Servo.Direction.REVERSE);
            robot.rightClaw.setPosition(0);
        }
        else if (clawPos == Hardware.ClawPos.STOP){
            robot.leftClaw.setPosition(0.5);
            robot.rightClaw.setPosition(0.5);
        }
    }

}
