package org.firstinspires.ftc.teamcode.helperclasses;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotVars;


public class ThreadPool {

    static ExecutorService pool = Executors.newFixedThreadPool(5);
    static Runnable c1 = new CheckPoint(300, 300, 2) {
        @Override
        public void onHit(){}
        };


    static Runnable robotVars = new RobotVars();


}
