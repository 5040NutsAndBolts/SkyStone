package org.firstinspires.ftc.teamcode.purepursuit;

public class LineSegment {
    private double x1;
    private double y1;
    private double x2;
    private double y2;

    public LineSegment(double x1, double y1, double x2, double y2) {
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;

    }

    public double getAngle() {
        return Math.atan2((y2 - y1), (x2 - x1));
    }

    public double getMagnitude() {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
}
