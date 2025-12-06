// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import edu.wpi.first.math.geometry.Translation2d;


/** Add your docs here. */
public class Polygon {
    private final List<Translation2d> m_points;

    public Polygon(ArrayList<Translation2d> points) {
        m_points = new ArrayList<Translation2d>();
        m_points.addAll(points);
    }

    public Polygon(List<Translation2d> points) {
        m_points = new ArrayList<Translation2d>();
        m_points.addAll(points);
    }

    public List<Translation2d> intersections(Line line) {
        List<Translation2d> intersections = new ArrayList<Translation2d>();

        final ListIterator<Translation2d> polyIt = m_points.listIterator(); //Getting an iterator along the polygon path
        Translation2d lastPoint = null;
        Translation2d startPoint = null;
        Translation2d startingPoint = null;
        if (polyIt.hasNext()) {
            startPoint = polyIt.next();
            startingPoint = startPoint;
        }
        while(polyIt.hasNext()) {
            lastPoint = polyIt.next();
            Double slope 
                = startPoint.getX() == lastPoint.getX() 
                ? Double.NaN
                : (lastPoint.getY() - startPoint.getY()) / (lastPoint.getX() - startPoint.getX());

            Line currentLine = new Line(startPoint, slope);
            Translation2d intersection = line.intersection(currentLine);
            if (intersection != null &&
                between(intersection.getX(), startPoint.getX(), lastPoint.getX()) &&
                between(intersection.getY(), startPoint.getY(), lastPoint.getY())
            ) {
                intersections.add(intersection);
            }
            startPoint = lastPoint;
        }
        
        // If polygon first point is not the last point, check that setgment too.
        if (startingPoint != null &&
            lastPoint != null &&
            (lastPoint.getX() != startingPoint.getX() || lastPoint.getY() != startingPoint.getY())) {
                lastPoint = startingPoint;
                Double slope 
                = startPoint.getX() == lastPoint.getX() 
                ? Double.NaN
                : (lastPoint.getY() - startPoint.getY()) / (lastPoint.getX() - startPoint.getX());
                Line currentLine = new Line(startPoint, slope);
                Translation2d intersection = line.intersection(currentLine);
                if (intersection != null&&
                    between(intersection.getX(), startPoint.getX(), lastPoint.getX()) &&
                    between(intersection.getY(), startPoint.getY(), lastPoint.getY())
                ) {
                    intersections.add(intersection);
                }
        }

        return intersections;

    }

    /**
     * between - determine if a value is within a range.
     * @param x - the value to test.
     * @param a - one end of the range.
     * @param b - the other end of the range.
     * @return - true if x is within the range.
     */
    public boolean between(double x, double a, double b) {
        return ((x >= Math.min(a,b)) && (x <= Math.max(a,b)));
    }

}
