package org.firstinspires.ftc.teamcode.competition.hardware;

import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class CapstoneDropper {

    private Hardware robot;

    private int pwm = 2502;

    ScheduledExecutorService executor =
            Executors.newSingleThreadScheduledExecutor();
    public CapstoneDropper(Hardware robot) {
        this.robot = robot;

    }

    Runnable decreasePWM = new Runnable() {
        public void run() {
            robot.capstoneDropper.setPosition(pwm--/2502);
            if(pwm<600) {
                executor.shutdownNow();
                executor = Executors.newSingleThreadScheduledExecutor();
            }


        }
    };

    /**
     * Create a new thread for the dropping mechanism
     */
    public void drop()
    {

        executor.scheduleAtFixedRate(decreasePWM, 0,14, TimeUnit.MILLISECONDS);

    }

}
