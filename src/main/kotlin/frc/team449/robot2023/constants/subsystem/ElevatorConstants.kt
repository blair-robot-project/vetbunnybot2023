package frc.team449.robot2023.constants.subsystem

import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.util.Color8Bit
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import kotlin.math.PI

object ElevatorConstants {

  /** NEO characteristics */
  const val EFFICIENCY = 0.90
  const val NOMINAL_VOLTAGE = 12.0
  const val STALL_TORQUE = 3.36 * EFFICIENCY
  const val STALL_CURRENT = 166.0
  const val FREE_CURRENT = 1.3
  val FREE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(5676.0)

  /** Mechanism2d Visual constants */
  const val ANGLE = 66.0
  const val MIN_LENGTH = 0.1
  const val WIDTH = 10.0
  val COLOR = Color8Bit(255, 0, 255)
  val DESIRED_COLOR = Color8Bit(0, 255, 0)

  const val LEFT_ID = 20
  const val LEFT_INVERTED = false
  const val RIGHT_ID = 21
  const val RIGHT_INVERTED = true
  const val EFFECTIVE_GEARING = 6.48 / 2
  const val PULLEY_RADIUS = 0.022352
  const val UPR = PULLEY_RADIUS * 2 * PI

  var kS = 0.0
  var kG = 1.075

  const val HIGH_DISTANCE = 1.4
  const val LOW_DISTANCE = 0.25
  const val STOW_DISTANCE = 0.0

  const val CURRENT_LIMIT = 60

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

  /** Elevator Sim constants. For carriage mass, it approximates both stages as a single stage */
  const val NUM_MOTORS = 2
  val CARRIAGE_MASS = 9.75

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