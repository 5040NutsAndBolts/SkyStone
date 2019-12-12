package org.firstinspires.ftc.teamcode.competition.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TowerArm {

    private Hardware robot;

    public TowerArm(Hardware hwMap) { robot = hwMap; }

    /**
     * Raising and lowering the tower arm at a pre-specified speed
     * @param height Which mode the function will run in
     */
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

    /**
     * Raising and lowering the tower arm at a user-specified speed
     * @param power Power at which to raise the tower arm
     */
    public void raiseLower(double power){
        robot.towerArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.towerArmMotor.setPower(power);
    }

    /**
     * Opening or closing the tower-grabbing claws
     * @param openClaw Whether or not to open the claw
     */
    public void openClose(boolean openClaw) {
        // HiTec servos can be programmed to a position within [.1, .9]
        // GoBuilda servos are programmed to a position within [0, 1]
        if (openClaw) {
            robot.clawRight.setPosition(.5);
            robot.clawLeft.setPosition(.5);
        }
        else {
            robot.clawRight.setPosition(1);
            robot.clawLeft.setPosition(1);
        }
        // TODO: Make it so I can almostClose() in this method, WHILE STILL HAVING 1 BOOLEAN ARGUMENT
    }

    /**
     * Set the claw to ALMOST close, used for grabbing the lowest position stone
     */
    public void almostClose() {
        robot.clawRight.setPosition(.8);
        robot.clawLeft.setPosition(.8);
    }
}
