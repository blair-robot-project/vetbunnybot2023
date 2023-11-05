package frc.team449.robot2023.constants.subsystem

import edu.wpi.first.wpilibj.util.Color8Bit

object ElevatorConstants {

  const val LEFT_ID = 20
  const val LEFT_INVERTED = false
  const val RIGHT_ID = 21
  const val RIGHT_INVERTED = true
  const val GEARING = 4.0
  const val PULLEY_RADIUS = 0.022352

  var kP = 3.0
  var kI = 0.0
  var kD = 0.0

  var kS = 0.0
  var kV = 3.475
  var kG = 1.269

  var MAX_VEL = (12 - kS) / kV
  const val MAX_ACC = 0.5

  const val HIGH_DISTANCE = 1.4
  const val LOW_DISTANCE = 0.25
  const val STOW_DISTANCE = 0.0

  const val CURRENT_LIMIT = 40

  /** Mechanism2d Visual constants */
  const val ANGLE = 66.0
  const val MIN_LENGTH = 0.1
  const val WIDTH = 10.0
  val COLOR = Color8Bit(255, 0, 255)

  /** Elevator Sim constants. For carriage mass, it approximates both stages as a single stage */
  const val NUM_MOTORS = 2
  val CARRIAGE_MASS = 11.0
}