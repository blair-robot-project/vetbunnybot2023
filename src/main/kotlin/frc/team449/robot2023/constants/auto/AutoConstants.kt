package frc.team449.robot2023.constants.auto

import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.PI

object AutoConstants {
  /** PID gains */
  const val DEFAULT_X_KP = 1.9
  const val DEFAULT_Y_KP = 1.9
  const val DEFAULT_ROTATION_KP = 2.0

  const val ORBIT_KP = 2 * PI
  val RED_WALL_DOUBLE = Translation2d(0.75, 6.13)
  val RED_FAR_DOUBLE = Translation2d(0.75, 7.465)

  /** Auto Balance PD Gains */
  const val AUTO_BAL_KP = 0.6075
  const val AUTO_BAL_KD = .0325
  const val ADJUST_SPEED = 2.125 // m/s
  const val MAX_ROT_VEL = 20.0 // deg/s

  const val Y_DISTANCE_BARRIER = 5.47878

  // in deg per 20 ms
  const val CONE_DROP_SWEEP_SPEED = 0.365
  const val CONE_DROP_SWEEP_TIME = 0.5

  const val CUBE_DROP_WAIT_BEFORE = 0.075
  const val CUBE_DROP_WAIT_AFTER = 0.2
}
