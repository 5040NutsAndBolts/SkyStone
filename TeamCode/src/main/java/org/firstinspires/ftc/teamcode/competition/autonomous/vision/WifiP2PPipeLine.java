package org.firstinspires.ftc.teamcode.competition.autonomous.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class WifiP2PPipeLine extends OpenCvPipeline {

    public Mat rawImage = new Mat();


    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(rawImage);
        return input;
    }
}
