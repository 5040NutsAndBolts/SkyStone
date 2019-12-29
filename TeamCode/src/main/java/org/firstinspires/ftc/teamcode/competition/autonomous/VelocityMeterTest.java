package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.competition.helperclasses.ThreadPool;
import org.firstinspires.ftc.teamcode.competition.helperclasses.VelocityMeter;

@Autonomous(group = "Auto", name = "velocity")
public class VelocityMeterTest extends LinearOpMode {

    private Hardware robot = new Hardware();
    private MecanumDrive drive = new MecanumDrive(robot);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        VelocityMeter vel = new VelocityMeter(robot);
        ThreadPool.pool.submit(vel);

        waitForStart();

        ElapsedTime e = new ElapsedTime();
        e.startTime();

        while (e.seconds() < 2 && opModeIsActive()) {
            drive.drive(1, 0, 0);
            telemetry.addData("Velocity", VelocityMeter.velocity);
            telemetry.addData("Angular Velocity", VelocityMeter.angularVelocity);
            telemetry.update();

        }
        vel.stop = true;
    }
}
