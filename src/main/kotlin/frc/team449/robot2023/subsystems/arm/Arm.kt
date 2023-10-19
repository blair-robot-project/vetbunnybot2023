package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.*
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.PI
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Controllable two-jointed arm
 * @param firstJoint main motor that moves the whole arm
 * @param secondJoint main motor at the joint that moves the second segment of the arm
 * @param feedForward the calculator for voltages based on a desired state
 * @param firstToSecondJoint length from the pivot motor to joint motor in METERS
 * @param secondJointToEndEffector length from the joint motor to the end-effector of the arm in METERS
 */
open class Arm(
  val firstJoint: WrappedMotor,
  val secondJoint: WrappedMotor,
  val firstJointEncoder: QuadEncoder,
  val secondJointEncoder: QuadEncoder,
  private val feedForward: TwoJointArmFeedForward,
  @field:Log val controller: ArmPDController,
  firstToSecondJoint: Double,
  secondJointToEndEffector: Double
) : Loggable, SubsystemBase() {

  /** visual of the arm as a Mechanism2d object */
  val visual = ArmVisual(
    firstToSecondJoint,
    secondJointToEndEffector,
    "Arm Visual :)"
  )

  /** kinematics that converts between (x, y) <-> (theta, beta) coordinates */
  val kinematics = ArmKinematics(
    firstToSecondJoint,
    secondJointToEndEffector
  )

  /** desired arm state */
  @Log.ToString
  var desiredState = ArmConstants.STOW

  /**
   * the current state of the arm in [ArmState]
   */
  @get:Log.ToString
  open val state: ArmState
    get() = ArmState(
      Rotation2d(MathUtil.inputModulus(firstJoint.position, -PI, PI)),
      Rotation2d(MathUtil.inputModulus(secondJointEncoder.position, -PI, PI)),
      firstJoint.velocity,
      secondJointEncoder.velocity
    )

  /**
   * The current state of the arm in [CartesianArmState]
   */
  @get:Log.ToString
  val coordinate: CartesianArmState
    get() = kinematics.toCartesian(state)

  fun setArmDesiredState(desiredState: ArmState) {
    val desState = ArmState(
      desiredState.theta,
      Rotation2d.fromDegrees(
        clamp(desiredState.beta.degrees, -156.8, 151.15)
      ),
      desiredState.thetaVel,
      desiredState.betaVel
    )
//     continue if state is same as last desired state
    if (desState == this.desiredState) {
      return
    }
    controller.reset()
    this.desiredState = desState
  }

  fun moveToState(newState: ArmState) {
    setArmDesiredState(newState)
    holdArm()
  }

  fun holdArm() {
    val ff = feedForward.calculate(state.matrix, false)
    val pid = controller.calculate(state.matrix, desiredState.matrix)
    val u = ff + pid
    firstJoint.setVoltage(u[0, 0])
    secondJoint.setVoltage(u[1, 0])
  }

  /**
   * Mitigate any speed on the joints
   */
  fun stop() {
    desiredState.thetaVel = 0.0
    desiredState.betaVel = 0.0
  }

  override fun periodic() {
    visual.setState(state, desiredState)
  }

  fun getClosestState(point: ArmState): ArmState? {
    var closestState: ArmState? = null
    var closestDistance = Double.MAX_VALUE

    for (state in ArmConstants.STATES) {
      val distanceToState = distanceBetweenStates(point, state)

      if (distanceToState < closestDistance) {
        closestDistance = distanceToState
        closestState = state
      }
    }

    return closestState
  }

  fun distanceBetweenStates(state1: ArmState, state2: ArmState): Double {
    val coordinate1 = kinematics.toCartesian(state1)
    val coordinate2 = kinematics.toCartesian(state2)
    return sqrt(
      (coordinate1.x - coordinate2.x).pow(2.0) + (coordinate1.z - coordinate2.z).pow(2.0)
    )
  }

  fun chooseTraj(endpoint: ArmState): ArmTrajectory? {
    val startPoint = getClosestState(this.desiredState)
    if (endpoint == startPoint) {
      this.desiredState = endpoint.copy()
      return null
    }
    if (startPoint == ArmConstants.CONE && endpoint == ArmConstants.CUBE) return ArmPaths.coneCube
    if (startPoint == ArmConstants.CUBE && endpoint == ArmConstants.CONE) return ArmPaths.cubeCone
    if (startPoint == ArmConstants.HIGH && endpoint == ArmConstants.MID) return ArmPaths.highMid
    if (startPoint == ArmConstants.MID && endpoint == ArmConstants.HIGH) return ArmPaths.midHigh
    if (startPoint == ArmConstants.LOW && endpoint == ArmConstants.HIGH) return ArmPaths.lowHigh
    if (startPoint == ArmConstants.LOW && endpoint == ArmConstants.MID) return ArmPaths.lowMid
    if (startPoint == ArmConstants.HIGH && endpoint == ArmConstants.LOW) return ArmPaths.highLow
    if (startPoint == ArmConstants.MID && endpoint == ArmConstants.LOW) return ArmPaths.midLow
    if (startPoint == ArmConstants.CUBE && endpoint == ArmConstants.THROW) return ArmPaths.cubeThrow
    if (startPoint == ArmConstants.THROW && endpoint == ArmConstants.CUBE) return ArmPaths.throwCube

    return if (startPoint == ArmConstants.BACK) {
      when (endpoint) {
        ArmConstants.SINGLE ->
          ArmPaths.backSingle
        ArmConstants.DOUBLE ->
          ArmPaths.backDouble
        ArmConstants.STOW ->
          ArmPaths.backStow
        ArmConstants.MID ->
          ArmPaths.backMid
        ArmConstants.CONE ->
          ArmPaths.backCone
        ArmConstants.CUBE ->
          ArmPaths.backCube
        ArmConstants.LOW ->
          ArmPaths.backLow
        else ->
          ArmPaths.backHigh
      }
    } else if (startPoint == ArmConstants.STOW) {
      when (endpoint) {
        ArmConstants.SINGLE ->
          ArmPaths.stowSingle
        ArmConstants.DOUBLE ->
          ArmPaths.stowDouble
        ArmConstants.BACK ->
          ArmPaths.stowBack
        ArmConstants.MID ->
          ArmPaths.stowMid
        ArmConstants.CONE ->
          ArmPaths.stowCone
        ArmConstants.CUBE ->
          ArmPaths.stowCube
        ArmConstants.LOW ->
          ArmPaths.stowLow
        else ->
          ArmPaths.stowHigh
      }
    } else if (endpoint == ArmConstants.STOW) {
      when (startPoint) {
        ArmConstants.AUTOMID ->
          ArmPaths.autoMidStow
        ArmConstants.SINGLE ->
          ArmPaths.singleStow
        ArmConstants.DOUBLE ->
          ArmPaths.doubleStow
        ArmConstants.MID ->
          ArmPaths.midStow
        ArmConstants.CUBE ->
          ArmPaths.cubeStow
        ArmConstants.CONE ->
          ArmPaths.coneStow
        ArmConstants.LOW ->
          ArmPaths.lowStow
        else ->
          ArmPaths.highStow
      }
    } else {
      when (startPoint) {
        ArmConstants.AUTOMID ->
          ArmPaths.autoMidBack
        ArmConstants.THROW ->
          ArmPaths.throwBack
        ArmConstants.SINGLE ->
          ArmPaths.singleBack
        ArmConstants.DOUBLE ->
          ArmPaths.doubleBack
        ArmConstants.MID ->
          ArmPaths.midBack
        ArmConstants.CONE ->
          ArmPaths.coneBack
        ArmConstants.CUBE ->
          ArmPaths.cubeBack
        ArmConstants.LOW ->
          ArmPaths.lowBack
        else ->
          ArmPaths.highBack
      }
    }
  }

  companion object {
    fun createArm(): Arm {
      val firstJointMotor = createSparkMax(
        "First Joint Motor",
        ArmConstants.FIRST_MOTOR_ID1,
        ArmEncoder.creator(
          ArmConstants.FIRST_ENCODER_CHAN,
          ArmConstants.FIRST_ENCODER_OFFSET,
          true
        ),
        slaveSparks = mapOf(
          ArmConstants.FIRST_MOTOR_ID2 to true
        ),
        currentLimit = ArmConstants.FIRST_JOINT_CURR_LIM,
        inverted = true,
        enableBrakeMode = true
      )

      val secondJointMotor = createSparkMax(
        "Second Joint Motor",
        ArmConstants.SECOND_MOTOR_ID,
        ArmEncoder.creator(
          ArmConstants.SECOND_ENCODER_CHAN,
          ArmConstants.SECOND_ENCODER_OFFSET,
          inverted = true
        ),
        currentLimit = ArmConstants.SECOND_JOINT_CURR_LIM,
        enableBrakeMode = true
      )

      val firstJointEncoder = QuadEncoder(
        "First joint quad",
        ArmConstants.FIRST_JOINT_QUAD_ENCODER,
        1024,
        2 * PI,
        1.0
      )

      val secondJointEncoder = QuadEncoder(
        "Second joint quad",
        ArmConstants.SECOND_JOINT_QUAD_ENCODER,
        1024,
        2 * PI,
        1.0
      )

      return Arm(
        firstJointMotor,
        secondJointMotor,
        firstJointEncoder,
        secondJointEncoder,
        TwoJointArmFeedForward.createFromConstants(),
        ArmPDController(
          ArmConstants.kP1,
          ArmConstants.kP2,
          ArmConstants.kD1,
          ArmConstants.kD2,
          ArmConstants.kI1,
          ArmConstants.kI2,
          ArmConstants.kErrDeadband
        ),
        ArmConstants.LENGTH_1,
        ArmConstants.LENGTH_2
      )
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.addDoubleProperty("First Joint Degrees", { Rotation2d.fromRadians(firstJoint.position).degrees }, null)
    builder.addDoubleProperty("Second Joint Degrees", { Rotation2d.fromRadians(secondJointEncoder.position).degrees }, null)
  }
}
