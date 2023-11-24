package frc.team449.robot2023.constants.subsystem

import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.util.Color8Bit
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import kotlin.math.PI

object ElevatorConstants {

  const val LEFT_ID = 20
  const val LEFT_INVERTED = false
  const val RIGHT_ID = 21
  const val RIGHT_INVERTED = true
  const val EFFECTIVE_GEARING = 7.0 / 2
  const val PULLEY_RADIUS = 0.022352
  const val UPR = PULLEY_RADIUS * 2 * PI

  var kP = 3.0
  var kI = 0.0
  var kD = 1.0

  var kS = 0.0
  var kV = 3.225
  var kG = 1.445
  var kSG = 1.445

  var MAX_VEL = (12 - kS) / kV
  const val MAX_ACC = 5.0

  const val HIGH_DISTANCE = 1.4
  const val LOW_DISTANCE = 0.25
  const val STOW_DISTANCE = 0.0

  const val CURRENT_LIMIT = 50

  // This is temporary, just so we can have two elevator subsystems to test out (internal smax and state space)
  val motor = createSparkMax(
    "Elevator Motor",
    LEFT_ID,
    NEOEncoder.creator(
      UPR,
      EFFECTIVE_GEARING
    ),
    enableBrakeMode = true,
    inverted = LEFT_INVERTED,
    currentLimit = CURRENT_LIMIT,
    slaveSparks = mapOf(
      Pair(RIGHT_ID, RIGHT_INVERTED)
    )
  )

  /** Mechanism2d Visual constants */
  const val ANGLE = 66.0
  const val MIN_LENGTH = 0.1
  const val WIDTH = 10.0
  val COLOR = Color8Bit(255, 0, 255)

  /** Elevator Sim constants. For carriage mass, it approximates both stages as a single stage */
  const val NUM_MOTORS = 2
  val CARRIAGE_MASS = 11.0

  /** State Space constants */
  val MODEL_POS_STDDEV = Units.inchesToMeters(3.0)
  val MODEL_VEL_STDDEV = Units.inchesToMeters(50.0)
  const val ENCODER_POS_STDDEV = 0.005
  val LQR_POS_TOL = Units.inchesToMeters(0.75)
  val LQR_VEL_TOL = Units.inchesToMeters(15.0)
  const val LQR_CONTROL_EFFORT_VOLTS = 10.5
  const val MAX_VOLTAGE = 10.5
  const val DT = 0.005
}