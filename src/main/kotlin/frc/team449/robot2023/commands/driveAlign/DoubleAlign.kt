package frc.team449.robot2023.commands.driveAlign

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPoint
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import frc.team449.control.auto.HolonomicFollower
import frc.team449.robot2023.Robot
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.auto.AutoConstants
import frc.team449.robot2023.constants.field.FieldConstants
import java.util.function.BooleanSupplier
import kotlin.math.PI
import kotlin.math.pow
import kotlin.math.sqrt

class DoubleAlign {
  /**
   * This class was just created so all the double substation command creation clutter
   *  isn't in ControllerBindings
   */

  /**
   * @param point The red alliance side point of the double subtation
   */
  private fun genTraj(
    point: Translation2d,
    isRed: Boolean,
    robot: Robot,
    endCondition: BooleanSupplier
  ): Command {

    val alliancePoint: Translation2d
    val endHeading: Rotation2d
    val endRotation: Rotation2d
    val xSpeedMin: Double
    val xSpeedMax: Double
    val ySpeedMin: Double
    val ySpeedMax: Double

    val fieldRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      robot.drive.desiredSpeeds,
      robot.drive.heading
    )

    if (isRed) {
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

    return HolonomicFollower(
      robot.drive,
      PathPlanner.generatePath(
        PathConstraints(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.DOUBLE_ALIGN_ACCEL),
        PathPoint(
          robot.drive.pose.translation,
          alliancePoint.minus(robot.drive.pose.translation).angle,
          robot.drive.pose.rotation,
          sqrt(
            MathUtil.clamp(
              fieldRelSpeeds.vxMetersPerSecond,
              xSpeedMin,
              xSpeedMax
            ).pow(2) +
              MathUtil.clamp(
                fieldRelSpeeds.vyMetersPerSecond,
                ySpeedMin,
                ySpeedMax
              ).pow(2)
          )
        ),
        PathPoint(alliancePoint, endRotation, endHeading)
      ),
      poseTol = Pose2d(0.05, 0.05, Rotation2d(0.025))
    ).until(endCondition)
  }

  fun rightDoubleAlign(robot: Robot, driveController: XboxController): Command {
    return ConditionalCommand(
      genTraj(AutoConstants.RED_FAR_DOUBLE, true, robot) { driveController.leftTriggerAxis >= 0.8 },
      genTraj(AutoConstants.RED_WALL_DOUBLE, false, robot) { driveController.leftTriggerAxis >= 0.8 }
    ) { RobotConstants.ALLIANCE_COLOR == DriverStation.Alliance.Red }
  }

  fun leftDoubleAlign(robot: Robot, driveController: XboxController): Command {
    return ConditionalCommand(
      genTraj(AutoConstants.RED_WALL_DOUBLE, true, robot) { driveController.leftTriggerAxis >= 0.8 },
      genTraj(AutoConstants.RED_FAR_DOUBLE, false, robot) { driveController.leftTriggerAxis >= 0.8 }
    ) { RobotConstants.ALLIANCE_COLOR == DriverStation.Alliance.Red }
  }
}
