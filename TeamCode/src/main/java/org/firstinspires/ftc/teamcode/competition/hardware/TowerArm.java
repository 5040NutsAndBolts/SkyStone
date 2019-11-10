package org.firstinspires.ftc.teamcode.competition.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class TowerArm {

    private Hardware robot;

    private int towerArmHeight = 1000;

    public TowerArm(Hardware hwMap) { robot = hwMap; }

    public void raiseLower(Hardware.TowerArmPos height){
        if(height == Hardware.TowerArmPos.RAISE) {
            robot.towerArmMotor.setTargetPosition(towerArmHeight);
            robot.towerArmMotor.setPower(.75);
        }
        else if(height == Hardware.TowerArmPos.LOWER) {
            robot.towerArmMotor.setTargetPosition(0);
            robot.towerArmMotor.setPower(-.75);
        }
        else if(height == Hardware.TowerArmPos.STOP){
            robot.towerArmMotor.setTargetPosition(robot.towerArmMotor.getCurrentPosition());
            robot.towerArmMotor.setPower(.1);
        }
    }

    public void openClose(Hardware.ClawPos clawPos) {

        if (clawPos == Hardware.ClawPos.OPEN) {
            // Left claw
            robot.leftClaw.setPower(.3);
            // Right claw
            robot.rightClaw.setPower(-.3);
        }
        else if (clawPos == Hardware.ClawPos.CLOSE) {
            // Left claw
            robot.leftClaw.setPower(-.3);
            // Right claw
            robot.rightClaw.setPower(.3);
        }
        else if (clawPos == Hardware.ClawPos.STOP){
            robot.leftClaw.setPower(0);
            robot.rightClaw.setPower(0);
        }
    }

}
