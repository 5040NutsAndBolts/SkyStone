package org.firstinspires.ftc.teamcode.competition.hardware;

import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;

public class CapstoneDropper {

    private Hardware robot;

    public boolean dropping = false;
    private int currentPWM = 2500;
    boolean drop;

    public CapstoneDropper(Hardware robot) {
        this.robot = robot;
        initThread();
    }

    /**
     * Create a new thread for the dropping mechanism
     */
    private void initThread() {
        currentPWM = 2500;
        robot.capstoneDropper.setPosition(1);

        Thread claw = new Thread() {
            @Override
            public void run() {
                while(true) {
                    if (dropping) {
                        robot.capstoneDropper.setPosition(currentPWM / 2502.0);
                        drop=!drop;
                        if(drop)
                            currentPWM-=1;
                    }
                    if (currentPWM < 900) {
                        dropping = false;
                        currentPWM = 2300;
                        robot.capstoneDropper.setPosition(1);
                    }
                }
            }
        };

        ThreadPool.pool.submit(claw);
    }
}
