/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import java.util.List;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Add your docs here.
 */
public class Utilities {
    

    public static double scaleDouble(final double input, final double to_min, final double to_max) {
        return scaleDouble(input, to_min, to_max, -1.0, 1.0);
    }

    public static double scaleDouble(final double input, final double to_min, final double to_max,
                                     final double from_min, final double from_max) {
        double x;
        double scaled_x = 0.0;
        if( to_max > to_min  && from_max > from_min )
        {
            x =  input;
            scaled_x = ((x - from_min) * (to_max - to_min)) / 
                        (from_max - from_min) +
                        to_min;
        }
        return scaled_x;    
    }

    public static void toSmartDashboard(String label, Pose2d pose) {
        SmartDashboard.putString(label, String.format("(%4.2f,%4.2f) %2.0f", pose.getX(), pose.getY(), pose.getRotation().getDegrees()) );
    }

    public static void toSmartDashboard(String label, List<Pose2d> poses) {
        String s = "";
        for(Pose2d p : poses) {
            s += String.format("(%4.2f,%4.2f,%3.0f),", p.getX(), p.getY(),p.getRotation().getDegrees());
        }
        SmartDashboard.putString(label, s );
    }

    public static Pose2d pose180(Pose2d pose) {
        Rotation2d rotate180 = new Rotation2d(Math.PI);
        Rotation2d newRotation = pose.getRotation().rotateBy(rotate180);
        return new Pose2d(pose.getX(), pose.getY(), newRotation);
    }

    // Create a new pose with the same X,Y coordinates rotated by <radians>
    public static Pose2d rotatePose(Pose2d pose, double radians) {
        //toSmartDashboard("RP Start", pose);
        Rotation2d rotation = new Rotation2d(radians);
        Rotation2d newRotation = pose.getRotation().rotateBy(rotation);
        //toSmartDashboard("RP End",new Pose2d(pose.getX(), pose.getY(), newRotation));
        return new Pose2d(pose.getX(), pose.getY(), newRotation);
    }

    // Create a new pose with the same X,Y coordinates and the specified rotation
    public static Pose2d setPoseAngle(Pose2d pose, double radians) {
        //toSmartDashboard("SP Start", pose);
        Rotation2d newRotation = new Rotation2d(radians);
        //toSmartDashboard("SP End",new Pose2d(pose.getX(), pose.getY(), newRotation));
        return new Pose2d(pose.getX(), pose.getY(), newRotation);
    }

    public static Pose2d backToPose(Pose2d pose, double distance) {
        Translation2d rotatedDistance = new Translation2d(distance, 0).rotateBy(pose.getRotation());
        return new Pose2d(pose.getX() + rotatedDistance.getX(), pose.getY() + rotatedDistance.getY(), pose.getRotation());
    }

    public static Pose2d facingNearPose(Pose2d pose, double distance) {
        Rotation2d rotate180 = new Rotation2d(Math.PI);
        Translation2d rotatedDistance = new Translation2d(distance, 0).rotateBy(pose.getRotation());
        Rotation2d newRotation = pose.getRotation().rotateBy(rotate180);
        return new Pose2d(pose.getX() + rotatedDistance.getX(), pose.getY() + rotatedDistance.getY(), newRotation);
    }
    // Return direction of turn for angle a->b->c
    // -1 if counter-clockwise
    //  0 if collinear
    //  1 if clockwise
    public static int ccw(Translation2d a, Translation2d b, Translation2d c) {
        double area2 = (b.getX() - a.getX()) * (c.getY() - a.getY()) - (c.getX() - a.getX()) * (b.getY() - a.getY());
        if      (area2 < 0) return -1;
        else if (area2 > 0) return +1;
        else                return  0;
    }

    // Use a winding algorithm to determine if the point is in the polugon defined by
    // the array of points.  The polygon must be closed meaning that the last point
    // in the array should be the same as the first point in the array.
    public static boolean pointInPolygon(Translation2d [] polygon, Translation2d point) {
        int winding = 0;
        for (int i = 0; i < polygon.length-1; i++) {
            int ccw = ccw(polygon[i], polygon[i+1], point);
            if (polygon[i+1].getY() >  point.getY() && point.getY() >= polygon[i].getY())  // upward crossing
                if (ccw == +1) winding++;
            if (polygon[i+1].getY() <= point.getY() && point.getY() <  polygon[i].getY())  // downward crossing
                if (ccw == -1) winding--;
        }
        return (winding != 0);
    }
    
    /**
     * refineYCoordinate - use the ultrasonic sensor to set the Pose estimator's
     * Y axis position on the field.  For this to make sense, the robot has to be
     * rotated to -90 degrees (ie back facing the amp wall) and near the 
     * amp wall.
    */
    public static void refineYCoordinate() {
        boolean reset = false;
        double ultrasonicRange = RobotContainer.m_PingResponseUltrasonicSubsystem.getRange();
        if (ultrasonicRange < 2.0 /* Meters */) {
            Pose2d currentPose = RobotContainer.m_poseEstimatorSubsystem.getCurrentPose();
            if (Math.abs(currentPose.getRotation().getDegrees() -  -90.0) < 5.0 /* degrees */) {
                reset = true;
                RobotContainer.m_poseEstimatorSubsystem.setCurrentPose(
                new Pose2d(
                    currentPose.getX(),
                    FieldConstants.yMax - ultrasonicRange - DriveConstants.kBackToCenterDistance,
                    currentPose.getRotation()
                )
                );
            }
            
        }
        // For debug, display whether we did reset the coordinate or not.
        SmartDashboard.putBoolean("refineY", reset);
    }
    
    // Boolean Supplier to return true if we are within 45 seconds of the end of the match.
    public static BooleanSupplier endGame = ()->DriverStation.getMatchTime() >= 2.5*60.0 - 45.0;
    
    // Check april tag coordinates for complementray tags.
    public static boolean comparePoses(Pose2d pose1, Pose2d pose2, double deltaXY, double deltaR) {
        Pose2d pose2Transformed = FieldConstants.BlueToRedPose(pose2);
        return  (Math.abs(pose1.getX() - pose2Transformed.getX()) < deltaXY) &&
                (Math.abs(pose1.getY() - pose2Transformed.getY()) < deltaXY) &&
                (Math.abs(pose1.getRotation().getRadians() - pose2Transformed.getRotation().getRadians()) < deltaR);
      }

      // Find the direction from point A to point B
      public static double getHeading(Translation2d a, Translation2d b) {
        double result;
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();
        result = Math.atan2(dy, dx);
        return result;
      }

    public static double least(double a, Double ... nums) {
        double result = a;
        for (Double num : nums) {
            if (num < result) {
                result = num;
            }
        }
        return result;
    }
    
    public static double greatest(double a, Double ... nums) {
        double result = a;
        for (Double num : nums) {
            if (num > result) {
                result = num;
            }
        }
        return result;
    }
}