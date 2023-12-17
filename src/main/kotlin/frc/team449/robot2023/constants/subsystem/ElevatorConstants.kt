package frc.team449.robot2023.constants.subsystem

import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.util.Color8Bit

object ElevatorConstants {
  const val EFFICIENCY = 0.875

  /** Mechanism2d Visual constants */
  const val ANGLE = 75.0
  const val MIN_LENGTH = 0.1
  const val WIDTH = 10.0
  val COLOR = Color8Bit(255, 0, 255)
  val DESIRED_COLOR = Color8Bit(0, 255, 0)

  const val LEFT_ID = 15
  const val LEFT_INVERTED = false
  const val RIGHT_ID = 61
  const val RIGHT_INVERTED = true
  const val EFFECTIVE_GEARING = 2 / 6.4
  const val PULLEY_RADIUS = 0.018415
  const val UPR = 0.11487263851412484

  // TODO: turn this on before comp
  const val BRAKE_MODE = false

  var kS = 0.895 - 0.2205
  var kG = 0.2205

  const val HIGH_DISTANCE = 1.114425
  const val LOW_DISTANCE = 0.5
  const val STOW_DISTANCE = 0.0

  const val CURRENT_LIMIT = 50
  const val PROFILE_CURR_LIM = 25

  // Mininum distance up the elevator to not hit the intake, calculated from CAD
  const val MIN_SAFE_POS = 0.485

  /** Elevator Sim constants. For carriage mass, it approximates both stages as a single stage */
  const val NUM_MOTORS = 2
  val CARRIAGE_MASS = 9.45

  /** State Space constants */
  val MODEL_POS_STDDEV = Units.inchesToMeters(3.0)
  val MODEL_VEL_STDDEV = Units.inchesToMeters(50.0)
  const val ENCODER_POS_STDDEV = 0.005
  val LQR_POS_TOL = 0.01
  val LQR_VEL_TOL = 0.075
  const val LQR_CONTROL_EFFORT_VOLTS = 12.0
  const val MAX_VOLTAGE = 12.0
  const val DT = 0.005
}
