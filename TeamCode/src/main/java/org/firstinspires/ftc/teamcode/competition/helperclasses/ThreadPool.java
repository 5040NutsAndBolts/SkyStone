package org.firstinspires.ftc.teamcode.competition.helperclasses;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.competition.autonomous.PurePursuitTest;
import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;


public class ThreadPool {

    static Hardware robot = new Hardware();

    public static ExecutorService pool = Executors.newFixedThreadPool(5);
    public static Runnable c1 = new CheckPoint(47,1,1,robot)
    {

        @Override
        public void onHit()
        {

            PurePursuitTest.point = false;

        }

    };



}
