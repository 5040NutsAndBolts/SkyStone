package org.firstinspires.ftc.teamcode.competition.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TowerArm {

    private Hardware robot;

    private boolean armOpen = true;

    public TowerArm(Hardware hwMap) { robot = hwMap; }

    public void raiseLower(Hardware.TowerArmPos height){
        if(height == Hardware.TowerArmPos.RAISE ){//&& robot.towerArmMotor.getCurrentPosition() > towerArmHeight) {
            robot.towerArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.towerArmMotor.setPower(.5);
        }
        else if(height == Hardware.TowerArmPos.LOWER) {
            robot.towerArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.towerArmMotor.setPower(-.5);
        }
        else if(height == Hardware.TowerArmPos.STOP) {
            robot.towerArmMotor.setTargetPosition(robot.towerArmMotor.getCurrentPosition());
            robot.towerArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.towerArmMotor.setPower(.5);
        }
        else if(height == Hardware.TowerArmPos.RESET) {
            robot.towerArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void openClose() {
        // HiTec servos can be programmed to a position within [.1, .9]
        // GoBuilda servos are programmed to a position within [0, 1]
        armOpen = !armOpen;
        if (armOpen) {
            robot.clawRight.setPosition(.5);
            robot.clawLeft.setPosition(.5);
        }
        else {
            robot.clawRight.setPosition(1);
            robot.clawLeft.setPosition(0);
        }
    }

    public void openLeft() {
        robot.clawLeft.setPosition(1);
    }

    public void openRight() {
        robot.clawRight.setPosition(0);
    }
}
