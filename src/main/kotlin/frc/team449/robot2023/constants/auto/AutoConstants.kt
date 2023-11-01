package frc.team449.robot2023.constants.auto

import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.PI

object AutoConstants {
  /** PID gains */
  const val DEFAULT_X_KP = 2.275
  const val DEFAULT_Y_KP = 2.275
  const val DEFAULT_ROTATION_KP = 2.0

  const val ORBIT_KP = 2 * PI
  val RED_WALL_DOUBLE = Translation2d(0.75, 6.13)
  val RED_FAR_DOUBLE = Translation2d(0.75, 7.465)

  /** Auto Balance PD Gains */
  const val AUTO_BAL_KP = 25.0
  const val ADJUST_SPEED = 1.5 // m/s
  const val MAX_ROT_VEL = 20.0 // deg/s

  const val Y_DISTANCE_BARRIER = 5.47878

  // in deg per 20 ms
  const val CONE_DROP_SWEEP_SPEED = 0.385
  const val CONE_DROP_SWEEP_TIME = 0.65
  const val CONE_WAIT = 0.075

  const val CUBE_DROP_WAIT_BEFORE = 0.0
  const val CUBE_DROP_WAIT_AFTER = 0.185
}
