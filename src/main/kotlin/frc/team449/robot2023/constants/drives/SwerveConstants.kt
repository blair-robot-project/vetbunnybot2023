package frc.team449.robot2023.constants.drives

import edu.wpi.first.math.util.Units

object SwerveConstants {
  /** Drive motor ports */
  const val DRIVE_MOTOR_FL = 1
  const val DRIVE_MOTOR_FR = 2
  const val DRIVE_MOTOR_BL = 3
  const val DRIVE_MOTOR_BR = 4
  const val TURN_MOTOR_FL = 11
  const val TURN_MOTOR_FR = 12
  const val TURN_MOTOR_BL = 13
  const val TURN_MOTOR_BR = 14

  /** Turning encoder channels */
  const val TURN_ENC_CHAN_FL = 8
  const val TURN_ENC_CHAN_FR = 7
  const val TURN_ENC_CHAN_BL = 9
  const val TURN_ENC_CHAN_BR = 6

  // BL 0.205474
  // BR 0.620525
  // FL 0.365650
  // FR 0.790104
  /** Offsets for the absolute encoders in rotations */
  const val TURN_ENC_OFFSET_FL = 0.365650 + .5
  const val TURN_ENC_OFFSET_FR = 0.540868
  const val TURN_ENC_OFFSET_BL = 0.205474 + .5
  const val TURN_ENC_OFFSET_BR = 0.620525 - .5

  /** PID gains for turning each module */
  const val TURN_KP = 0.8
  const val TURN_KI = 0.0
  const val TURN_KD = 0.0

  /** Feed forward values for driving each module */
  const val DRIVE_KS = 0.18183
  const val DRIVE_KV = 2.6237
  const val DRIVE_KA = 0.42824

  /** PID gains for driving each module*/
  const val DRIVE_KP = 0.4
  const val DRIVE_KI = 0.0
  const val DRIVE_KD = 0.0

  /** Drive configuration */
  const val DRIVE_GEARING = 1 / 6.75
  const val DRIVE_UPR = 0.31818905832
  const val TURN_UPR = 2 * Math.PI
  const val MAX_ATTAINABLE_MK4I_SPEED = (12 - DRIVE_KS) / DRIVE_KV
  const val DRIVE_CURRENT_LIM = 55
  const val STEERING_CURRENT_LIM = 40

  /** Wheelbase = wheel-to-wheel distance from front to back of the robot */
  /** Trackwidth = wheel-to-wheel distance from side to side of the robot */
  val WHEELBASE = Units.inchesToMeters(24.75) // ex. FL to BL
  val TRACKWIDTH = Units.inchesToMeters(21.75) // ex. BL to BR
}
