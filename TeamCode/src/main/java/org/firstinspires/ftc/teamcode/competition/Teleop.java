package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.competition.hardware.*;


@TeleOp(name="Teleop", group="Teleop")
public class Teleop extends OpMode {

    private Hardware robot;
    private MecanumDrive driveTrain;
    private IntakeMech intake;
    private TowerArm towerArm;
    private CapstoneMech capstoneMech;
    private GrabbingMech grabbingMech;

    /**
     * Instantiates objects
     */
    public Teleop() {
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);
        intake = new IntakeMech(robot);
        towerArm = new TowerArm(robot);
        capstoneMech = new CapstoneMech(robot);
        grabbingMech = new GrabbingMech(robot);
    }

    /**
     * Method run on init to initialize hardware
     */
    public void init() { robot.init(hardwareMap); }

    /**
     * The loop played during the game
     */
    @Override
    public void loop() {
        robot.updatePositionRoadRunner();
        telemetry.addData("Intake speed", intake.intakeSpeed);
        telemetry.addLine("=========");
        telemetry.addData("X Position", robot.x);
        telemetry.addData("Y Position", robot.y);
        telemetry.addData("Rotation", robot.theta);
        telemetry.update();

        // Top Half (Gamepad 2)
            // Raising/lowering the tower arm
                if (gamepad2.right_stick_y > .01 || gamepad2.right_stick_y < .01)
                    towerArm.raiseLower(gamepad2.right_stick_y);
                else
                    towerArm.raiseLower(0);

            // Raising the capstone placing mechanism
                if(gamepad2.y)
                    capstoneMech.moveSlidesUp();
                else if (gamepad2.a)
                    capstoneMech.moveSlidesDown();
                else
                    capstoneMech.holdSlides();

            // Controlling the speed of the intake
                if(gamepad2.b) {
                    intake.intakeSpeed = .25;
                }
                else if (gamepad2.a) {
                    intake.intakeSpeed = .5;
                }
                else {
                    intake.intakeSpeed = 1;
                }

            // Foundation down button
                if (gamepad2.y) {
                    grabbingMech.grabFoundation();
                }

        // Bottom Half (Gamepad 1)
            // Intake/Outtake
                if(gamepad1.right_trigger>.01)
                    intake.setPower(1);
                else if(gamepad1.left_trigger>.01)
                    intake.setPower(0);
                else
                    intake.setPower(2);

            // Grabbing mechanism
                if(gamepad1.left_bumper)
                    grabbingMech.resetFoundation();
                if(gamepad1.right_bumper)
                    grabbingMech.grabFoundation();

            // Stone guide
                if(gamepad1.start)
                    intake.guideOut();
                if(gamepad1.back)
                    intake.guideIn();


                if (gamepad1.a)
                    towerArm.raiseLower(-.05);

            // Drive Train
                if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0)
                    driveTrain.hardBrakeMotors();
                else
                    driveTrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
    }
}
