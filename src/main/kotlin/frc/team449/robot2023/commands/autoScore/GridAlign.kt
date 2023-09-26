package frc.team449.robot2023.commands.autoScore

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.*
import frc.team449.control.DriveCommand
import frc.team449.robot2023.commands.arm.ArmSweep
import frc.team449.robot2023.commands.driveAlign.ProfiledPoseAlign
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.field.FieldConstants
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import frc.team449.robot2023.subsystems.arm.control.ArmState
import kotlin.math.PI
import kotlin.math.abs

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
    Levels.MID to abs(robot.arm.kinematics.toCartesian(ArmConstants.MID).x + ArmConstants.PIECE_TO_WHEEL) -
      ArmConstants.backToArmBase - FieldConstants.midNodeFromEdge,
    Levels.HIGH to abs(robot.arm.kinematics.toCartesian(ArmConstants.HIGH).x + ArmConstants.PIECE_TO_WHEEL) -
      ArmConstants.backToArmBase - FieldConstants.highNodeFromEdge
  )

  private fun inArmTolerance(current: ArmState, goal: ArmState, tolerance: ArmState): Boolean {
    val error1 = current.theta.radians - goal.theta.radians
    val error2 = current.beta.radians - goal.beta.radians

    val there = abs(error1) < abs(tolerance.theta.radians) &&
      abs(error2) < abs(tolerance.beta.radians) &&
      abs(current.thetaVel) < abs(tolerance.thetaVel) &&
      abs(current.betaVel) < abs(tolerance.betaVel)

    println(there)

    return there
  }

  fun autoScore(
    target: FieldConstants.TargetPosition,
    isRed: Boolean,
    level: Levels,
    isConeNode: Boolean,
    tolerance: Pose2d = Pose2d(0.065, 0.065, Rotation2d.fromDegrees(1.25)),
    armTolerance: ArmState = ArmState(Rotation2d.fromDegrees(3.0), Rotation2d.fromDegrees(3.0), 0.2, 0.2)
  ): Command {
    println("doing traj generation here")

    val alliancePoint: Translation2d
    val endHeading: Rotation2d
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

    val armTraj = robot.arm.chooseTraj(levelsToArm[level]!!)

    val profiledPath = ProfiledPoseAlign(
      robot.drive,
      Pose2d(alliancePoint, endHeading),
      MathUtil.clamp(
        fieldRelSpeeds.vxMetersPerSecond,
        xSpeedMin,
        xSpeedMax
      ),
      MathUtil.clamp(
        fieldRelSpeeds.vyMetersPerSecond,
        ySpeedMin,
        ySpeedMax
      ),
      tolerance = tolerance
    )

    val waitTimeForArm = 0.85

    return SequentialCommandGroup(
      ParallelCommandGroup(
        profiledPath,
        SequentialCommandGroup(
          WaitCommand(waitTimeForArm),
          ArmFollower(robot.arm) { armTraj }
        )
      ),
      ConditionalCommand(
        SequentialCommandGroup(
          ParallelRaceGroup(
            WaitCommand(0.5),
            ArmSweep(
              robot.arm,
              { 1.0 },
              Rotation2d.fromDegrees(10.0)
            )
          ),
          InstantCommand(robot.endEffector::pistonRev)
        ),
        InstantCommand(robot.endEffector::autoReverse)
      ) { isConeNode },
      InstantCommand(robot.endEffector::stop),
      ParallelRaceGroup(
        SequentialCommandGroup(
          WaitCommand(0.35),
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.BACK) }
        ),
        DriveCommand(robot.drive, robot.oi)
      )
    )
  }
}
