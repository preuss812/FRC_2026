package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utilities;
import frc.robot.AutonomousPlans;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This class provide a cpmplex command that verifies that the robot 
 * posistion is close to the expected position.
 * If the position is not close, it sleeps for 15 seconds to
 * run out the clock on the autonomous period without taking any action.
 */
public class VerifyStartingPositionCommand extends SequentialCommandGroup {
    
    public VerifyStartingPositionCommand(
        PoseEstimatorSubsystem poseEstimator,
        int autoMode)
    {
        /* the intent here is to:
            *  - wait until we see an april tag
            *  - wait 1 second to allow the coordinates to be fully updated.
            *  - compare the current robot pose from the pose estimator to the expected pose.
            *  - do almost nothing if the poses are too far apart.
            */
        double closeEnoughXY = Units.inchesToMeters(12.0); // Accept a starting position within 1 foot of expected.
        double closeEnoughTheta = Units.degreesToRadians(10.0); // Accept a starting rotation within 10 degrees of expected.

        final Pose2d expectedPose = AutonomousPlans.waypoints.get(autoMode)[0];

        addCommands(
            new ConditionalCommand(
                new WaitCommand(0.001), // Wait 1ms if we are close to the correct starting point, basically a NOP
                new WaitCommand(15), // Wait long enough to make sure autonomous is over and no action is taken.
                () -> Utilities.comparePoses(expectedPose, poseEstimator.getCurrentPose(), closeEnoughXY, closeEnoughTheta)
            )
        );
    }
}
