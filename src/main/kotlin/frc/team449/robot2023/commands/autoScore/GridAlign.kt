package frc.team449.robot2023.commands.autoScore

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPoint
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.*
import frc.team449.control.auto.HolonomicFollower
import frc.team449.robot2023.commands.arm.ArmSweep
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.field.FieldConstants
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import frc.team449.robot2023.subsystems.arm.control.ArmState
import java.util.function.BooleanSupplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot

class GridAlign(
  private val robot: frc.team449.robot2023.Robot
) {
  enum class Levels {
    LOW,
    MID,
    HIGH
  }

  private val levelsToArm: Map<Levels, ArmState> = mapOf(
    Levels.LOW to ArmConstants.LOW,
    Levels.MID to ArmConstants.MID,
    Levels.HIGH to ArmConstants.HIGH
  )

  private val levelsToOffsets: Map<Levels, Double> = mapOf(
    Levels.LOW to 0.0,
    Levels.MID to abs(robot.arm.kinematics.toCartesian(ArmConstants.MID).x - 0.035) -
      ArmConstants.backToArmBase - FieldConstants.midNodeFromEdge,
    Levels.HIGH to abs(robot.arm.kinematics.toCartesian(ArmConstants.HIGH).x - 0.035) -
      ArmConstants.backToArmBase - FieldConstants.highNodeFromEdge
  )

  fun autoScore(
    target: FieldConstants.TargetPosition,
    isRed: Boolean,
    endCondition: BooleanSupplier,
    level: Levels,
    isConeNode: Boolean,
    tolerance: Pose2d = Pose2d(0.075, 0.075, Rotation2d.fromDegrees(1.75))
  ): Command {
    println("doing traj generation here")

    val alliancePoint: Translation2d
    val endHeading: Rotation2d
    val endRotation: Rotation2d
    val xSpeedMin: Double
    val xSpeedMax: Double
    val ySpeedMin: Double
    val ySpeedMax: Double

    val point = Pose2d(
      FieldConstants.PlacementPositions[target]!!.x + levelsToOffsets[level]!!,
      FieldConstants.PlacementPositions[target]!!.y,
      FieldConstants.PlacementPositions[target]!!.rotation
    )

    val fieldRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      robot.drive.desiredSpeeds,
      robot.drive.heading
    )

    if (!isRed) {
      alliancePoint = Translation2d(point.x, point.y)
      endHeading = Rotation2d()
      endRotation = Rotation2d(PI)
      xSpeedMin = -RobotConstants.MAX_LINEAR_SPEED
      xSpeedMax = 0.0
      if (robot.drive.pose.y < alliancePoint.y) {
        ySpeedMin = 0.0
        ySpeedMax = RobotConstants.MAX_LINEAR_SPEED
      } else {
        ySpeedMin = -RobotConstants.MAX_LINEAR_SPEED
        ySpeedMax = 0.0
      }
    } else {
      alliancePoint = Translation2d(FieldConstants.fieldLength - point.x, point.y)
      endHeading = Rotation2d(PI)
      endRotation = Rotation2d()
      xSpeedMin = 0.0
      xSpeedMax = RobotConstants.MAX_LINEAR_SPEED
      if (robot.drive.pose.y < alliancePoint.y) {
        ySpeedMin = 0.0
        ySpeedMax = RobotConstants.MAX_LINEAR_SPEED
      } else {
        ySpeedMin = -RobotConstants.MAX_LINEAR_SPEED
        ySpeedMax = 0.0
      }
    }

    val path = PathPlanner.generatePath(
      PathConstraints(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.DOUBLE_ALIGN_ACCEL),
      PathPoint(
        robot.drive.pose.translation,
        alliancePoint.minus(robot.drive.pose.translation).angle,
        robot.drive.pose.rotation,
        hypot(
          MathUtil.clamp(
            fieldRelSpeeds.vxMetersPerSecond,
            xSpeedMin,
            xSpeedMax
          ),
          MathUtil.clamp(
            fieldRelSpeeds.vyMetersPerSecond,
            ySpeedMin,
            ySpeedMax
          )
        )
      ),
      PathPoint(alliancePoint, endRotation, endHeading)
    )

    val armTraj = robot.arm.chooseTraj(levelsToArm[level]!!)

    val waitTimeForArm = if (path.totalTimeSeconds > (armTraj?.totalTime ?: 0.0) + 1.25) path.totalTimeSeconds - (armTraj?.totalTime ?: 0.0) - 1.25 else 0.0

    return SequentialCommandGroup(
      ParallelCommandGroup(
        HolonomicFollower(
          robot.drive,
          path,
          poseTol = tolerance,
          timeout = 4.0
        ),
        ParallelCommandGroup(
          WaitCommand(waitTimeForArm),
          ArmFollower(robot.arm) { armTraj }
        )
      ),
      ConditionalCommand(
        SequentialCommandGroup(
          ParallelRaceGroup(
            WaitCommand(0.35),
            ArmSweep(
              robot.arm,
              { 1.0 },
              Rotation2d.fromDegrees(15.0)
            )
          ),
          InstantCommand(robot.endEffector::pistonRev)
        ),
        InstantCommand(robot.endEffector::autoReverse)
      ) { isConeNode },
      InstantCommand(robot.endEffector::stop)
    )
  }
}
