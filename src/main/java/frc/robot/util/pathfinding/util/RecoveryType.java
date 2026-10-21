package frc.robot.util.pathfinding.util;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.pathfinding.builders.GoingMerry;
import frc.robot.util.pathfinding.commands.PPRecoveryCommand;

import java.util.function.BiFunction;

//*Enum determining the type of path recovery followed in ShinFollowPathCommand
// NO_RECOVERY: No extra recovery mode, basically acts like a normal FollowPathCommand
// DRIVE_TO_POINT_RECOVERY: Uses a simple drive-to-point command for recovery. Reliable but can get caught in narrow areas
// PATHFINDING_RECOVERY: Pathfinds to the desired state. Most resource heavy, but guarantees robot safety (hopefully)
public enum RecoveryType {
    NO_RECOVERY((path, state) -> Commands.none()),
    DRIVE_TO_POINT_RECOVERY((path, state) -> PPRecoveryCommand.generateCommand(path, state)),
    PATHFINDING_RECOVERY((path, state) -> GoingMerry.pathfindToPose(state.pose, path.getGlobalConstraints(), state.linearVelocity));

    public final BiFunction<PathPlannerPath, PathPlannerTrajectoryState, Command> recoveryCommandBuilder;

    private RecoveryType(BiFunction<PathPlannerPath, PathPlannerTrajectoryState, Command> recoveryCommandBuilder) {
        this.recoveryCommandBuilder = recoveryCommandBuilder;
    }

}