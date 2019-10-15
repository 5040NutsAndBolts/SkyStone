package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.competition.Hardware;
import org.firstinspires.ftc.teamcode.competition.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.competition.helperclasses.HelperMethods;

import static java.lang.Math.abs;

/**
 * Testing out the odometry
 */
@Autonomous(name = "OdometryTest", group = "Auto")
public class OdometryTest extends LinearOpMode {

    private Hardware robot;
    private MecanumDrive driveTrain;

    private enum TestAuto {
        FORWARD_CM,
        FORWARD_CM_PID,
        MOVE_TO_POINT
    }

    @Override
    public void runOpMode() {
        // Sets up classes
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);

        TestAuto autoToRun = TestAuto.FORWARD_CM_PID;

        // Initializes robot
        robot.init(hardwareMap);

        // Waits until "start" or "stop" is pressed
        while(!isStarted()&&!isStopRequested()){
            robot.bulkData = robot.expansionHub.getBulkInputData();
            robot.updatePosition();
            if(gamepad1.x)
                autoToRun = TestAuto.FORWARD_CM;
            if(gamepad1.a)
                autoToRun = TestAuto.FORWARD_CM_PID;
            if(gamepad1.b)
                autoToRun = TestAuto.MOVE_TO_POINT;

            telemetry.addData("auto to run", autoToRun);
            telemetry.update();
            robot.resetPosition();
            robot.resetEncoders();
        }

        // If testing new driveTrain.forwardCm(cm)
        if(autoToRun == TestAuto.FORWARD_CM){
            // Testing new driveTrain.forwardCm(30) since it doesn't work
            double endpointX = robot.x + 75;

            // Create variables for if robot is stuck and won't move forward
            int stuckTimer = 0;
            double lastStuckX = robot.x;
            long endTime = System.currentTimeMillis() + (long)(1.5 * 1000000);

            double motorPower = .1;

            // Loop to try and move robot forward
            while(opModeIsActive() && (robot.x < endpointX)) {
                robot.bulkData = robot.expansionHub.getBulkInputData();
                robot.updatePosition();

                if(System.currentTimeMillis() >= endTime){
                    //break;
                }

                // Check if robot is stuck in same position
                if(HelperMethods.inThreshhold(lastStuckX, robot.x, .25))
                    stuckTimer++;
                else {
                    lastStuckX = robot.x;
                    stuckTimer = 0;
                }
                // If robot hasn't moved within last 1000 updates
                if (stuckTimer >= 1000)
                    break;

                if(motorPower>1)
                    motorPower = 1;

                // Try and move robot forward
                driveTrain.drive(motorPower,0,0);

                motorPower += .1;

                telemetry.addData("X", robot.x);
                telemetry.addData("Y", robot.y);
                telemetry.addData("Theta", robot.theta);
                telemetry.addData("Power", motorPower);
                telemetry.update();
            }

            driveTrain.powerSet(0);
        }

        // If testing new driveTrain.forwardCmPid(cm, tolerance)
        else if (autoToRun == TestAuto.FORWARD_CM_PID){
            // Testing new driveTrain.forwardCmPid(30)
            double endpointX = robot.x + 75;
            double power = .1;

            while(opModeIsActive() && robot.x < endpointX && power != 0){
                robot.bulkData = robot.expansionHub.getBulkInputData();
                robot.updatePosition();
                telemetry.addData("X", robot.x);
                telemetry.addData("Y", robot.y);
                telemetry.addData("Theta", robot.theta);
                telemetry.addLine("===== Motor Powers =====");
                telemetry.addData("LeftFront", (robot.leftFront.getPower()));
                telemetry.addData("RightFront", (robot.rightFront.getPower()));
                telemetry.addData("LeftRear", (robot.leftRear.getPower()));
                telemetry.addData("RightRear", (robot.rightRear.getPower()));
                telemetry.update();

                // Slow down as closer to point
                if(Math.abs(endpointX - robot.x) <= 5)
                    power = Math.abs(endpointX - robot.x) * .2;
                // Speed up to avoid instant acceleration issues
                else if (power < 1)
                    power *= 1.2;
                else
                    power = 1;

                driveTrain.drive(power, 0,0);
            }
        }

        // If testing new driveTrain.moveToPoint(x, y, theta)
        else {

        }

        while(opModeIsActive()){
            robot.updatePosition();
            telemetry.addData("X", robot.x);
            telemetry.addData("Y", robot.y);
            telemetry.addData("Theta", robot.theta);
            telemetry.update();
        }
    }
}
