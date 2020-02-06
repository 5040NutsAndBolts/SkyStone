package org.firstinspires.ftc.teamcode.helperclasses;

public class HelperMethods {

    // Makes class unable to be instantiated
    private HelperMethods() {}

    /**
     * Easy way of determining if a VALUE1 is within THRESHOLD PERCENT of a VALUE2
     *
     * @param value1           First value
     * @param value2           Second value
     * @param thresholdPercent Percentage value for which value1/value2 needs to be within
     * @return If the two values are within the threshold percent of each other
     */
    public static boolean inThreshold(double value1, double value2, double thresholdPercent) {
        double ratio = value1 / value2;

        boolean withinUpperThreshold = ratio <= 1 + thresholdPercent * .01;
        boolean withinLowerThreshold = ratio >= 1 - thresholdPercent * .01;

        return withinUpperThreshold && withinLowerThreshold;
    }

    /**
     * Determines the quadrant an angle is in
     * Doesn't work if angle is between two quadrants
     * @param angle Angle in question
     * @return The quadrant the given angle is within
     */
    public static int getQuadrant(double angle) {
        if (angle > 0 && angle < Math.PI / 2.0)
            return 1;
        if (angle > Math.PI / 2.0 && angle < Math.PI)
            return 2;
        if (angle > Math.PI && angle < 3.0 * Math.PI / 2.0)
            return 3;
        return 4;
    }

    /**
     * Returns the value of the angle within 2 Pi
     * @param angle Angle in question
     * @return Value of angle within 2 Pi
     */
    public static double angleWrap(double angle) {
        return angle % (Math.PI * 2);
    }

    /**
     * Returns if a point is within a given circle
     * @param otherPoint Point in question
     * @param centerPoint Center point of the circle
     * @param radius Radius of the circle
     * @return If the point in question is within the circle
     */
    public static boolean withinCircle(Point otherPoint, Point centerPoint, double radius) {
        return
                Math.pow(otherPoint.x - centerPoint.x, 2) + Math.pow(otherPoint.y - centerPoint.y, 2)
                        <= Math.pow(radius, 2);
    }

    public static void main(String[] args) {
        System.out.println(inThreshold(0, -24, 4));
    }
}