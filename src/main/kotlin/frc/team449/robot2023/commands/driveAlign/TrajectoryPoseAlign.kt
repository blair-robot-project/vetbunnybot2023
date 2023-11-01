package frc.team449.robot2023.commands.driveAlign

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPoint
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.auto.HolonomicFollower
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2023.constants.auto.AutoConstants
import io.github.oblarg.oblog.annotations.Config
import java.util.function.Supplier

/**
 * @param drive The holonomic drive you want to align with
 * @param targetPose The pose you want to drive up to
 * @param xController The profiled PID controller with constraints you want to use for fixing X error
 * @param yController The profiled PID controller with constraints you want to use for fixing Y error
 * @param thetaController The non-Profiled PID controller you want to use for fixing rotational error
 * @param poseTolerance The allowed tolerance from the targetPose
 * @param timeout Maximum time to wait after the estimated time for the trajectory to finish is done
 * @param maxSpeeds Maximum speeds and acceleration to run the path at
 */
class TrajectoryPoseAlign(
  private val drive: HolonomicDrive,
  private val currPose: Supplier<Pose2d>,
  private val targetPose: Pose2d,
  @field:Config.PIDController(name = "Pose Align X PID") var xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Pose Align Y PID") var yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Pose Align Rotation PID") var thetaController: PIDController = PIDController(
    AutoConstants.DEFAULT_ROTATION_KP,
    0.0,
    0.0
  ),
  private val poseTolerance: Pose2d = Pose2d(0.05, 0.05, Rotation2d(0.05)),
  private val timeout: Double = 0.75,
  private val maxSpeeds: PathConstraints = PathConstraints(3.0, 1.5)
) {

  fun generateCommand(): Command {
    // Generate a PathPlanner trajectory on the fly
    val traj = PathPlanner.generatePath(
      maxSpeeds,
      PathPoint(currPose.get().translation, Rotation2d(), currPose.get().rotation),
      PathPoint(targetPose.translation, Rotation2d(), targetPose.rotation)
    )

    // Return a command that follows the trajectory
    return HolonomicFollower(
      drive,
      traj,
      xController,
      yController,
      thetaController,
      poseTolerance,
      timeout,
      resetPose = false
    )
  }
}
