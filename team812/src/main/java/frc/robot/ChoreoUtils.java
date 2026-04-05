package frc.robot;

import java.util.List;
import java.util.Optional;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

public class ChoreoUtils {
    /*
     * mirrorY - helper function for mirroring trajectories across the field center line.
     * @param trajectory - (Optional<Trajectory<SwerveSample>>) the trajectory to mirror.
     * @return  - (Optional<Trajectory<SwerveSample>>) the mirrored trajectory, or empty if the input was empty.
     */
    public static Optional<Trajectory<SwerveSample>> mirrorY(Optional<Trajectory<SwerveSample>> trajectory) {
        if (trajectory.isEmpty()) {
            return trajectory;
        }
        Trajectory<SwerveSample> traj = trajectory.get();
        List<SwerveSample> mirroredSamples = traj.samples().stream()
            .map(s -> new SwerveSample(
                    s.t,
                    s.x,
                    Constants.FieldConstants.fieldWidth - s.y,
                    -s.heading,
                    s.vx,
                    -s.vy,
                    -s.omega,
                    s.ax,
                    -s.ay,
                    -s.alpha,
                    s.moduleForcesX(),
                    s.moduleForcesY()
            ))
            .toList();
        return Optional.of(new Trajectory<>(traj.name() + "_mirrored", mirroredSamples, traj.splits(), traj.events()));
    }
}
