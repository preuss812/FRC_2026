package frc.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class Line {
    // Slope-intercept form y = mX + b
    public double m;
    public double b;
    // points on the line
    public double y1;
    public double x1;
    // Standard formula for the line, Ax + By + C = 0
    public double A;
    public double B;
    public double C;
    public boolean verticalLine;

    public Line(Translation2d point, double m) {
        // We now have Point-Slope form, (Y- y1) = m(X - x1) 
        this.m = m;
        this.x1 = point.getX();
        this.y1 = point.getY();
        // We now have Point-Slope form, 

        // Convert to standard form, AX + BY + C = 0
        if (Double.isNaN(m)) verticalLine = true;
        if (!verticalLine) {
            A = -m;
            B = 1;
            C = - (A*x1 + B*y1);
        } else {
            // Verfical line 
            A = 1;
            B = 0;
            C = x1;
        }

        // Calculate the slope intercept for y = mX + b
        if (!Double.isNaN(m)) {
            b = -m*x1 + y1;
        }

    }
    
    /**
     * find the intersection of one line with another.
     * @param (Line) the line to intersect with this line.
     * @return (Translation2d) - the point at which the 2 lines intersect or null.
     */
    public Translation2d intersection(Line l2) {
        Translation2d intersection;

        if (Double.isNaN(l2.m) && !Double.isNaN(this.m)) {
            // l2 is vertical line
            double x = l2.x1;
            double y = this.m * x + this.b;
            intersection = new Translation2d(x, y);
            return intersection;
        } else if (Double.isNaN(this.m) && !Double.isNaN(l2.m)) {
            // this is vertical line
            double x = this.x1;
            double y = l2.m * x + l2.b;
            intersection = new Translation2d(x, y);
            return intersection;
        } else if (this.m != l2.m) {
            // Formula from https://www.cuemath.com/geometry/intersection-of-two-lines/
            intersection = new Translation2d(
                (this.B*l2.C - l2.B*this.C)/(this.A*l2.B - l2.A*this.B),
                (this.C*l2.A - l2.C*this.A)/(this.A*l2.B - l2.A*this.B)
            );
            return intersection;
        } else if (this.b == l2.b) { // the same line
            // Pick an arbitrary point on the line
            intersection = new Translation2d(l2.x1, l2.m * l2.x1 + l2.b);
            return intersection;
        }
        return null; // intersection is either everywhere or nowhere, they have the same slope

    }

    public double distanceFromPointToLine(Translation2d point) {
        // Formula from https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        double distance = Math.abs(A*point.getX() + B*point.getY() + C) / (Math.sqrt(Math.pow(A,2)+ Math.pow(B,2)));
        return distance;
    }

    public static double distanceBetweenTwoPoints(Translation2d a, Translation2d b) {
        // d = sqrt(dx**2+dy**2)
        double distance = Math.pow(Math.pow(a.getX() - b.getX(),2.0) + Math.pow(a.getY() - b.getY(), 2), 0.5);
        return distance;
    }

    public static double vectorLength(Translation2d vector) {
        double distance =  Math.pow(Math.pow(vector.getX(),2.0) + Math.pow(vector.getY() - vector.getY(), 2), 0.5);
        return distance;
    }

    public static Translation2d scaleVector(Translation2d a, Translation2d b, double length) {
        double magnitude = distanceBetweenTwoPoints(a,b);
        if (magnitude == 0.0) {
            return new Translation2d(0.0,0.0);
        }
        double rescale = length/magnitude;
        return new Translation2d(
            a.getX() + (a.getX() - b.getX())*rescale,
            a.getY() + (a.getY() - b.getY())*rescale);
    }
}