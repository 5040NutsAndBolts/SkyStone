package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.competition.helperclasses.HelperMethods;


@Autonomous(group="Auto",name = "trapezoid")
public class trapezoidPowerAuto extends AutoMethods {

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        //double point_x_wanted, point_y_wanted;
        //trapizodal(1,1);

        waitTime(3);
        while (!isStarted()) {
            updateOdometryTeleop();
            drive.softBrakeMotors();
        }

        double startPos = robot.x;
        double endPos = 24;
        while (opModeIsActive() && !HelperMethods.inThreshold(robot.x, endPos, 30)) {
            updateOdometryTeleop();
            double forwardMovement = -trapezoidPowerFunc(startPos, robot.x, endPos);
            drive.drive(forwardMovement, 0,0);
        }
        drive.drive(0,0,0);
        displayEndAuto();

    }

    /*public double trapizodal(double point_x_wanted, double point_y_wanted){
        robot.updatePositionRoadRunner();

        double distance_to_travel = Math.sqrt(Math.pow((point_x_wanted - robot.x), 2) + Math.pow((point_y_wanted - robot.y), 2));
        double startX = robot.x;
        double startY = robot.y;
        double decell_distance_start = .9 * distance_to_travel;
        double maxspeed_distance_start = distance_to_travel * .1;
        double constantAccel = .5 / maxspeed_distance_start;
        double movementSpeed = 0;
        double currentDistance = 0;

        while (currentDistance < distance_to_travel) {
            robot.updatePositionRoadRunner();
            double current_position_x = robot.x;
            double current_position_y = robot.y;

            currentDistance = Math.sqrt(Math.pow((current_position_x - startX), 2) + Math.pow((current_position_y - startY), 2));

            if (maxspeed_distance_start > currentDistance) {
                movementSpeed = movementSpeed + constantAccel;
                return movementSpeed;
            }
            else if (currentDistance >= maxspeed_distance_start && currentDistance < decell_distance_start) {
                movementSpeed = 1;
                return movementSpeed;
            }
            else if (currentDistance >= decell_distance_start) {

                movementSpeed = movementSpeed - constantAccel;
                return movementSpeed;
            }
            else{
                return movementSpeed = 0;
            }
        }

        return movementSpeed;
    }*/

    private double trapezoidPowerFunc(double startVal, double currentVal, double endVal) {
        double accelEndPercent = .1;
        double decelStartPercent = .3;
        double minimumPower = .1;
        double maxPower = .75;

        double currentPercent = (currentVal-startVal)/(endVal-startVal);

        if (currentPercent < accelEndPercent) {
            return currentPercent/accelEndPercent + minimumPower;
        }
        else if (currentPercent > decelStartPercent) {
            return 1.1 - currentPercent;
        }
        else if (currentPercent > accelEndPercent && currentPercent < decelStartPercent)
            return maxPower;
        return 0;
    }
}