package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This class provide a cpmplex command that verifies that the robot has seen an april tag and
 * has verified that it's calculated posistion is close to the expected position.
 */
public class VerifyExpectedAprilTagCommand extends SequentialCommandGroup {
    public static boolean match(int a, int b) { return a == b;}

    public VerifyExpectedAprilTagCommand(
        PoseEstimatorSubsystem poseEstimator,
        int allianceID,
        int aprilTagID)
    {

        addCommands(
            new ConditionalCommand(
                new WaitCommand(0.001), // Wait 1ms if we are close to the correct starting point, basically a NOP
                new WaitCommand(15), // Wait long enough to make sure autonomous is over and no action is taken.
                () -> match(poseEstimator.lastAprilTagSeen(),FieldConstants.allianceAprilTag(allianceID, aprilTagID))
            )
        );
    }
}
