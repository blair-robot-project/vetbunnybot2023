package frc.team449.control.auto

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2023.auto.AutoConstants
import io.github.oblarg.oblog.annotations.Config
import kotlin.math.abs

class ChoreoRoutine(
  @field:Config.PIDController(name = "X PID") var xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Y PID") var yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Rotation PID") var thetaController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0),
  private val drive: HolonomicDrive,
  private val stopEventMap: HashMap<Int, Command>,
  private val trajectories: MutableList<ChoreoTrajectory>,
  private val poseTol: Pose2d = Pose2d(0.05, 0.05, Rotation2d(0.05)),
  private val resetPosition: Boolean = false,
  private val resetPositionTolerance: Pose2d = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
  private val timeout: Double = 0.65
) {

  private fun resetPose(trajectory: ChoreoTrajectory): Command {
    val poseError = drive.pose.relativeTo(trajectory.initialPose)

    if (abs(poseError.x) < resetPositionTolerance.x &&
      abs(poseError.y) < resetPositionTolerance.y &&
      abs(poseError.rotation.radians) < resetPositionTolerance.rotation.radians
    ) {
      return PrintCommand("Pose not reset.")
    }

    return InstantCommand({ drive.pose = trajectory.initialPose })
  }

  fun createRoutine(): Command {
    val commands = SequentialCommandGroup(
      stopEventMap[0],
      resetPose(trajectories[0])
    )

    for (i in 0..trajectories.size) {
      commands.addCommands(
        ChoreoFollower(
          drive,
          trajectories[i],
          hashMapOf(),
          xController,
          yController,
          thetaController,
          poseTol,
          timeout,
          resetPosition
        )
      )
      commands.addCommands(stopEventMap[i + 1])
    }

    return commands
  }

}