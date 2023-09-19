package frc.team449.robot2023.constants

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import frc.team449.robot2023.constants.drives.SwerveConstants
import kotlin.math.PI

object RobotConstants {

  /** Other CAN ID */
  const val PDH_CAN = 49

  /** Controller Configurations */
  const val RATE_LIMIT = 3.5 * PI
  const val TRANSLATION_DEADBAND = .125
  const val ROTATION_DEADBAND = .125

  /** Drive configuration */
  const val MAX_LINEAR_SPEED = SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED // m/s
  const val MAX_ROT_SPEED = PI // rad/s
  const val MAX_ACCEL = 14.75 // m/s/s
  val INITIAL_POSE = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0))

  const val DOUBLE_ALIGN_ACCEL = 2.25

  /** PID controller for Orthogonal turning */
  val ORTHOGONAL_CONTROLLER = ProfiledPIDController(
    2.25,
    0.0,
    0.0,
    TrapezoidProfile.Constraints(
      MAX_ROT_SPEED,
      RATE_LIMIT
    )
  )

  var ALLIANCE_COLOR: DriverStation.Alliance = DriverStation.getAlliance()

  val IR_CHANNEL = 15

  // Robot dimensions (INCLUDING BUMPERS)
  val ROBOT_WIDTH = Units.inchesToMeters(27.0 + 3.25 * 2)
  val ROBOT_LENGTH = Units.inchesToMeters(30.0 + 3.25 * 2)
}
