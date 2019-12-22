package org.firstinspires.ftc.teamcode.competition.helperclasses;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;




public class ThreadPool {

    private ArrayList<Thread> tasks = new ArrayList<>();
    public static ExecutorService pool = Executors.newFixedThreadPool(5);



}
