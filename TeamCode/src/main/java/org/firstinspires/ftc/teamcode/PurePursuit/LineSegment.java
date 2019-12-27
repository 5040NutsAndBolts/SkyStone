package org.firstinspires.ftc.teamcode.PurePursuit;

public class LineSegment {
    double x1 = 0;
    double y1 = 0;
    double x2;
    double y2;

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
