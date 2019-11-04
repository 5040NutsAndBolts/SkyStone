package org.firstinspires.ftc.teamcode.competition.hardware;

public class TowerArm {

    private Hardware robot;

    private int towerArmHeight = 50;
    private double leftClawPos = 100;
    private double rightClawPos = 100;
    private boolean armOpen;

    public TowerArm(Hardware hwMap) {
        robot = hwMap;
        armOpen = false;
    }

    public void raiseLower(Hardware.TowerHeight height){
        if(height == Hardware.TowerHeight.RAISE) {
            robot.towerArmMotor.setTargetPosition(towerArmHeight);
            robot.towerArmMotor.setPower(.25);
        }
        else if(height == Hardware.TowerHeight.LOWER) {
            robot.towerArmMotor.setTargetPosition(0);
            robot.towerArmMotor.setPower(.25);
        }
        else {
            robot.towerArmMotor.setTargetPosition(robot.towerArmMotor.getCurrentPosition());
            robot.towerArmMotor.setPower(0);
        }
    }

    public void openClose(){
        armOpen = !armOpen;

        if(armOpen) {
            robot.leftClaw.setPosition(leftClawPos);
            robot.rightClaw.setPosition(rightClawPos);
        }
        else {
            robot.leftClaw.setPosition(0);
            robot.rightClaw.setPosition(0);
        }
    }

}
