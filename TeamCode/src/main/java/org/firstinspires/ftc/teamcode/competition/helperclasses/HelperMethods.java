package org.firstinspires.ftc.teamcode.competition.helperclasses;

import java.util.HashMap;

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
     * @return
     */
    public static boolean inThreshhold(double value1, double value2, double thresholdPercent) {
        double ratio = value1 / value2;

        boolean withinUpperThreshold = ratio >= 1 + thresholdPercent * .01;
        boolean withinLowerThreshold = ratio <= 1 - thresholdPercent * .01;

        return withinUpperThreshold || withinLowerThreshold;
    }

    /**
     * Allows the user to convert between any of the following units and vice versa:
     * Inches and Feet
     * Inches and Centimeters
     * Inches and Millimeters
     * Feet and Centimeters
     * Feet and Millimeters
     * Centimeters and Millimeters
     *
     * @param conversionString string for conversion to happen (ex: 15cm to in)
     * @return converted number
     */
    public static double convertUnits(String conversionString) {
        HashMap<String, Double> conversionDictionary = new HashMap<String, Double>() {{
            // inches and feet
            put("in to ft", 1 / 12.0);
            put("ft to in", 12.0);
            // inches and centimeters
            put("in to cm", 2.54);
            put("cm to in", 1 / 2.54);
            // inches and millimeters
            put("in to mm", 25.4);
            put("mm to in", 1 / 25.4);
            // feet and centimeters
            put("ft to cm", 1 / 30.48);
            put("cm to ft", 30.48);
            // feet and millimeters
            put("ft to mm", 1 / 304.8);
            put("mm to ft", 304.8);
            // centimeters and millimeters
            put("cm to mm", 1 / 10.0);
            put("mm to cm", 10.0);
        }};

        double convertedNumber = 0;

        // Replaces all non-numeric,
        String numberToBeConverted = conversionString.replaceAll("[\\s+a-zA-Z : '-']", "");
        double unconvertedNumber = Double.parseDouble(numberToBeConverted);
        //
        String conversionKey = conversionString.replaceAll("[0-9\\-.]", "");

        try {
            convertedNumber = unconvertedNumber * conversionDictionary.get(conversionKey);
        } catch (Exception e) {
            System.out.println(conversionString);
            System.out.println(conversionString.replaceAll("[0-9\\-.]", ""));
            System.out.println(numberToBeConverted);
            System.out.println(conversionKey);
            System.out.println(e);
        }

        return convertedNumber;
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
        System.out.println(convertUnits("12in to ft"));
    }
}
