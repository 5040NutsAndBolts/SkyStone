package org.firstinspires.ftc.teamcode.competition.helperclasses;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;


public class ThreadPool {

    public static Hardware robot = new Hardware();

    public static ExecutorService pool = Executors.newCachedThreadPool();







}
