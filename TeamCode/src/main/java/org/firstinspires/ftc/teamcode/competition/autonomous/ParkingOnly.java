package org.firstinspires.ftc.teamcode.competition.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;

@Autonomous(group="Auto",name = "RightAuto")
public class ParkingOnly extends LinearOpMode {

    private Hardware robot = new Hardware();
    private MecanumDrive drive = new MecanumDrive(robot);

    private boolean movingRight = true;
    private boolean parkOnWall = true;
    private int waitTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while (!isStarted()) {
            if (movingRight)
                telemetry.addLine("Robot will move right (relative to human)");
            else
                telemetry.addLine("Robot will move left (relative to human)");
            telemetry.addLine("==========");
            if (parkOnWall)
                telemetry.addLine("Robot will park on wall");
            else
                telemetry.addLine("Robot will NOT park on wall");
            telemetry.addLine("==========");
            telemetry.addData("Wait time (in seconds)", waitTime);
            telemetry.addLine("==========");
            telemetry.addData("X Position", robot.x);
            telemetry.addData("Y Position", robot.y);
            telemetry.addData("Rotation", robot.theta);
            telemetry.update();

            if (gamepad1.y)
                parkOnWall = false;
            if (gamepad1.a)
                parkOnWall = true;
            if (gamepad1.x)
                movingRight = false;
            if (gamepad1.b)
                movingRight = true;
            if (gamepad1.dpad_up)
                waitTime += .25;
            if (gamepad1.dpad_down)
                waitTime -= .25;

            if (waitTime > 25)
                waitTime = 25;
            if (waitTime < 0)
                waitTime = 0;
        }

        // Waiting before the robot actually does anything
        waitTime *= 1000;
        long endTime = System.currentTimeMillis() + waitTime;
        while (System.currentTimeMillis() < endTime);

        // Moving the claw out of the way
        endTime = System.currentTimeMillis() + 270;
        long dropTime = System.currentTimeMillis() + 200;
        while(System.currentTimeMillis() < endTime && opModeIsActive()) {
            if(System.currentTimeMillis() > dropTime);{
                robot.clawLeft.setPosition(.7);
                robot.clawRight.setPosition(.7);
            }
            robot.towerArmMotor.setPower(.7);
        }
        robot.towerArmMotor.setPower(-.2);

    }
}
