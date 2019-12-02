package org.firstinspires.ftc.teamcode.competition.autonomous.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class GrayscaleFilter {
    private Scalar lowerColor, upperColor;

    public GrayscaleFilter(int lower, int upper) {
        this.lowerColor = new Scalar(lower);
        this.upperColor = new Scalar(upper);
    }

    public void process(Mat input, Mat mask) {
        // Convert the input to grayscale
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2GRAY);

        // Applies a gaussian blur to the image
        // TODO: Check if image even needs to be blurred
        Imgproc.GaussianBlur(input,input,new Size(5,5),0);

        // Checks if values from input are within boundaries and applies that to the mask
        Core.inRange(input, lowerColor, upperColor, mask);

        // Deletes modified input data to save space and time
        // TODO: Check if this even affects performance or changes anything
        input.release();
    }
}
