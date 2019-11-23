package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.acmerobotics.dashboard.canvas.Spline;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.competition.Hardware;
import org.firstinspires.ftc.teamcode.competition.drivetrain.MecanumDrive;

@Autonomous(name = "Road Runner Test", group = "Auto")
public class RoadRunnerTest extends LinearOpMode
{

    private Hardware robot;
    private MecanumDrive driveTrain;
    @Override
    public void runOpMode() {
        // Sets up classes
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);

        // Initializes robot
        robot.init(hardwareMap);

        // Waits until "start" or "stop" is pressed
        while (!isStarted() && !isStopRequested()) {
            robot.bulkData = robot.expansionHub.getBulkInputData();
            robot.updatePositionRoadRunner();

            telemetry.addData("x: ", robot.x);
            telemetry.addData("y: ", robot.y);

            telemetry.update();
        }
        LineSegment line = new LineSegment(
                new Vector2d(2, 2),
                new Vector2d(14, 14)
        );
        LinearInterpolator interp = new LinearInterpolator(
                Math.toRadians(30), Math.toRadians(45)
        );
        PathSegment segment = new PathSegment(line, interp);
        Path path = new Path(segment);
        PIDCoefficients translationalPid = new PIDCoefficients(7, 0, 0);
        PIDCoefficients headingPid = new PIDCoefficients(4, 0, 0);
        HolonomicPIDVAFollower follower = new HolonomicPIDVAFollower(translationalPid, translationalPid, headingPid);

        DriveConstraints constraints = new DriveConstraints(20, 40, 80, 1, 2, 4);


        while(opModeIsActive())
        {
            Trajectory traj = TrajectoryGenerator.INSTANCE.generateTrajectory(path, constraints);

            follower.followTrajectory(traj);
            // call in loop
            DriveSignal signal = follower.update(new Pose2d(robot.x, robot.y,robot.prevHeading));
            telemetry.addData("forward",signal.component1().component1());
            telemetry.addData("sideways",signal.component1().component2());
            telemetry.addData("angle",signal.component1().component3());
            telemetry.addData("x: ", robot.x);
            telemetry.addData("y: ", robot.y);
            telemetry.update();

        }
    }

}
