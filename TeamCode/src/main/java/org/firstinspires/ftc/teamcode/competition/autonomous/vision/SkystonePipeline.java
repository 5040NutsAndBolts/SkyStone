package org.firstinspires.ftc.teamcode.competition.autonomous.vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * Notes:
 * - The camera will never send more than one frame to be processed at a time and thus,
 *       and thus, processFrame() will never be called multiple times simultaneously
 * - Processing is NOT invoked on the OpMode thread, but rather the frame worker thread
 * - For whatever reason instantiated objects cannot access public methods or fields other than
 *       the two overridden methods, but can be accessed if they are set to static: so they are
 */
/**
 * OpenCV pipeline used to find the skystone
 * Detects skystone by finding the largest/darkest area in a given camera frame
 */
public class SkystonePipeline extends OpenCvPipeline {

    // Coordinate position of the top left corner of the selected rectangle
    public static Point screenPosition = new Point(0,0);

    private Mat
            rawImage,       // Raw image output from the camera
            workingMat,     // The image currently being worked on and being modified
            selectionMask,  // Image mask for all found rectangles
            reflectiveMask, // Image mask of all reflective objects within a certain threshold
            skystoneMask,   // Image mask to only show the selected skystone
            hierarchy;      // Hierarchy mask needed by OpenCV for contour finding

    // Grayscale filter to make it easier for computer to find very dark colors
    private GrayscaleFilter grayscaleFilter;

    // Current selected rectangle
    private Rect skystoneRect;

    // Variable to hold types of stages the viewport will render in
    private enum ViewportRenderStage {
        REFLECT_THRESHOLD,  // Pure black and pure white image of most reflective objects in camera
        RAW_IMAGE,          // This will show only what the camera sees (before anything is done to it)
        SELECTIONS,         // This will show all selected rectangles and the current selected rectangle
        SKYSTONE_ONLY,      // Selects only the skystone
    }
    // Sets the current stage to render
    public static ViewportRenderStage stageToRenderToViewport = ViewportRenderStage.SELECTIONS;
    // Array to cycle through of each viewport render stage
    private ViewportRenderStage[] stages;

    /**
     * Sets up all the variables to keep code clean
     */
    public SkystonePipeline() {
        rawImage = new Mat();
        workingMat = new Mat();
        selectionMask = new Mat();
        reflectiveMask = new Mat();
        reflectiveMask = new Mat();
        skystoneMask = new Mat();
        hierarchy = new Mat();
        grayscaleFilter = new GrayscaleFilter(0, 25);
        skystoneRect = new Rect();
        stages = ViewportRenderStage.values();
    }

    /**
     * When the viewport is tapped it changes what is rendered on the screen:
     * Raw Image (Just regular camera), Display Mat (Highlights boxes and shows chosen), and
     * Threshold (What the "vision" sees)
     *     AKA it turns all dark colors to full white and all non-dark colors to full black
     */
    @Override
    public void onViewportTapped() {
        int nextStageNum = stageToRenderToViewport.ordinal() + 1;

        if(nextStageNum >= stages.length)
            nextStageNum = 0;

        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    public Mat processFrame(Mat input) {
        // Copies the original input to other materials to be worked on so they aren't overriding each other
        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(selectionMask);
        input.copyTo(reflectiveMask);
        input.copyTo(skystoneMask);

        // Sets the best fitting rectangle to the one currently selected
        Rect bestRect = skystoneRect;

        // Numerical value for the "best fit" rectangle
        // MAX_VALUE to find the lesser difference
        double lowestScore = Double.MAX_VALUE;

        // Draws a rectangle on the blackMask at location(top left, bottom right)
        // with a color of RGB(255,255,255)
        //     that has a thickness multiplier of 1
        // without using a thick anti-aliased line (all you need to know is this saves the most time)
        //     really just a 4-connected Bresenham algorithm line type
        Imgproc.rectangle(reflectiveMask, bestRect.tl(), bestRect.br(), new Scalar(255,255,255), 1, Imgproc.LINE_4);

        // processes the working image into a grayscale image
        //     Uses a clone of the working image because the image gets modified in the process
        grayscaleFilter.process(workingMat.clone(), reflectiveMask);

        // Creates a list for all contoured objects the camera will find
        List<MatOfPoint> contoursList = new ArrayList<>();

        // Finds contours and saves them to the reflective mask and contour list
        //     Uses the RETR_TREE as a method for determining contours
        //     Uses CHAIN_APPROX_SIMPLE for approximating the edging of contours better
        // TODO: Test if hierarchy mask is needed at all or if it can be replaced with "new Mat()"
        Imgproc.findContours(reflectiveMask, contoursList, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        // Draws outlines of contours from contoursList onto the selectionMask
        //     Drawn in gray
        Imgproc.drawContours(selectionMask, contoursList,-1, new Scalar(40,40,40),2);

        // Scores all the contours and selects the best of them
        for(MatOfPoint contour : contoursList){
            // Calculate the "score" of the selected contour
            double score = calculateScore(contour);

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(contour);

            // Draw the current found rectangle on the selections mask
            //     Drawn in blue
            Imgproc.rectangle(selectionMask, rect.tl(), rect.br(), new Scalar(0,0,255),2);

            // If the result is better then the previously tracked one, set this rect as the new best
            if(score < lowestScore && rect.tl().y >= 120&&rect.tl().x>70){
                lowestScore = score;
                bestRect = rect;
            }
        }

        // Draw the "best fit" rectangle on the selections mask and skystone only mask
        //     Drawn in red
        Imgproc.rectangle(selectionMask, bestRect.tl(), bestRect.br(), new Scalar(255,0,0),4);
        Imgproc.rectangle(skystoneMask, bestRect.tl(), bestRect.br(), new Scalar(255,0,0), 4);

        // Sets the position of the selected rectangle (relative to the screen resolution)
        screenPosition = new Point(bestRect.x, bestRect.y);

        // Sets the "best fit" rectangle to be the chosen skystone
        skystoneRect = bestRect;

        // Sets which viewport render to use
        switch (stageToRenderToViewport) {
            case REFLECT_THRESHOLD:
                // Ensures reflective mask is actually grayscale
                Imgproc.cvtColor(reflectiveMask, reflectiveMask, Imgproc.COLOR_GRAY2BGR);
                return reflectiveMask;
            case RAW_IMAGE:
                return rawImage;
            case SKYSTONE_ONLY:
                return skystoneMask;
            case SELECTIONS:
            default:
                return selectionMask;
        }
    }

    /**
     * "scores" a given section, "input," of the screen
     * The "score" being the area of a given section on the screen
     * @param input a section of the screen which needs to be scored
     * @return calculated "score" of a given section on the screen
     */
    private double calculateScore(Mat input) {
        // Validates input, returning the maximum value if invalid
        if(!(input instanceof MatOfPoint))
            return Double.MAX_VALUE;
        // Otherwise returns the calculated area of the contour
        return -Imgproc.contourArea(input);
    }
}