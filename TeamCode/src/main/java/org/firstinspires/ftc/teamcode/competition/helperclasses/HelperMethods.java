package org.firstinspires.ftc.teamcode.competition.helperclasses;

public class HelperMethods {

    // Makes class unable to be instantiated
    private HelperMethods() {
    }

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

    public static int getQuadrant(double angle) {
        if (angle > 0 && angle < Math.PI / 2.0)
            return 1;
        if (angle > Math.PI / 2.0 && angle < Math.PI)
            return 2;
        if (angle > Math.PI && angle < 3.0 * Math.PI / 2.0)
            return 3;
        return 4;
    }

    public static double angleWrap(double angle) {

        while (angle >= 2 * Math.PI) {
            angle -= 2 * Math.PI;
        }

        while (angle < 0) {
            angle += 2 * Math.PI;
        }

        return angle;

    }

    public static void main(String[] args) {
        System.out.println(inThreshold(0, -24, 4));
    }
}
