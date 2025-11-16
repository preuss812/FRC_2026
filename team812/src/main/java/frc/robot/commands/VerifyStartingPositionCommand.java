package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utilities;
import frc.robot.AutonomousPlans;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.AllianceConfigurationSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This class provide a cpmplex command that verifies that the robot has seen an april tag and
 * has verified that it's calculated posistion is close to the expected position.
 * If the tag is not found, it just sleeps for 15 seconds to prevent any action during autonomous.
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

        final Pose2d expectedPose = AllianceConfigurationSubsystem.isBlueAlliance() ? 
            AutonomousPlans.waypoints.get(autoMode)[0] :
            FieldConstants.BlueToRedPose(AutonomousPlans.waypoints.get(autoMode)[0]) ;

        addCommands(
            new ConditionalCommand(
                new WaitCommand(0.001), // Wait 1ms if we are close to the correct starting point, basically a NOP
                new WaitCommand(15), // Wait long enough to make sure autonomous is over and no action is taken.
                () -> Utilities.comparePoses(expectedPose, poseEstimator.getCurrentPose(), closeEnoughXY, closeEnoughTheta)
            )
        );
    }
}
