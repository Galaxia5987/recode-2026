package frc.robot.autonomous

import com.pathplanner.lib.path.PathConstraints
import frc.robot.migrationError
import org.wpilib.command3.Command

var useOdometryOnlyInAuto = false

private fun runPath(
    path: String,
    mirror: Boolean = false,
    pathfindBefore: Boolean = false,
    pathfindingConstraints: PathConstraints? = null,
): Command {
    /*
    var path = PathPlannerPath.fromPathFile(path)
    if (mirror) {
        path = path.mirrorPath()
    }

    if (pathfindBefore) {
        return AutoBuilder.pathfindThenFollowPath(
            path,
            pathfindingConstraints!!
        )
    }

    return AutoBuilder.followPath(path)

     */
    migrationError()
}

private fun runPathAndReset(
    pathName: String,
    mirror: Boolean = false,
): Command {
    /*
    val path = PathPlannerPath.fromPathFile(pathName)
    var startPose = path.startingHolonomicPose.get()
    if (mirror) {
        startPose = startPose.mirror()
    }

    return Commands.runOnce({
            BetterPoseEstimator.getInstance()
                .resetPose(startPose.flipIfNeeded())
        })
        .andThen(runPath(pathName, mirror))
     */
    migrationError()
}
