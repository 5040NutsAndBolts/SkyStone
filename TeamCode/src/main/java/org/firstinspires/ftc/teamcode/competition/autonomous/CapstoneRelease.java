package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group="Auto",name="capstone")
public class CapstoneRelease extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo capstoneRelease;
        capstoneRelease=hardwareMap.servo.get("capstone");
        waitForStart();


        ElapsedTime time = new ElapsedTime();
        time.startTime();
        capstoneRelease.setPosition(1);
        while(time.seconds()<.5);
        capstoneRelease.setPosition(0);

        while(time.seconds()<2);
    }
}
