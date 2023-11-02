package frc.team449.control.holonomic

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.constants.RobotConstants
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot

class SwerveOrthogonalCommand(
  private val drive: SwerveDrive,
  private val controller: XboxController,
  private val fieldOriented: () -> Boolean = { true }
) : CommandBase() {

  private var prevX = 0.0
  private var prevY = 0.0

  private var prevTime = 0.0

  private var dx = 0.0
  private var dy = 0.0
  private var magAcc = 0.0
  private var dt = 0.0
  private var magAccClamped = 0.0

  private var rotScaled = 0.0
  private val allianceCompensation = { if (RobotConstants.ALLIANCE_COLOR == DriverStation.Alliance.Red) 0.0 else PI }
  private val directionCompensation = { if (RobotConstants.ALLIANCE_COLOR == DriverStation.Alliance.Red) -1.0 else 1.0 }

  private var atGoal = true

  private var rotRamp = SlewRateLimiter(RobotConstants.RATE_LIMIT)

  private val timer = Timer()

  private val rotCtrl = RobotConstants.ORTHOGONAL_CONTROLLER

  private var skewConstant = 0.5

  init {
    addRequirements(drive)
    rotCtrl.enableContinuousInput(-PI, PI)
    rotCtrl.setTolerance(0.025)
  }

  override fun initialize() {
    timer.restart()

    prevX = drive.currentSpeeds.vxMetersPerSecond
    prevY = drive.currentSpeeds.vyMetersPerSecond
    prevTime = 0.0
    dx = 0.0
    dy = 0.0
    magAcc = 0.0
    dt = 0.0
    magAccClamped = 0.0

    rotRamp = SlewRateLimiter(
      RobotConstants.RATE_LIMIT,
      -RobotConstants.RATE_LIMIT,
      drive.currentSpeeds.omegaRadiansPerSecond
    )

    var atGoal = true
  }

  override fun execute() {
    val currTime = timer.get()
    dt = currTime - prevTime
    prevTime = currTime

    val xScaled = (if (abs(controller.leftY) < RobotConstants.TRANSLATION_DEADBAND) .0 else -controller.leftY) *
      drive.maxLinearSpeed
    val yScaled = (if (abs(controller.leftX) < RobotConstants.TRANSLATION_DEADBAND) .0 else -controller.leftX) *
      drive.maxLinearSpeed

    dx = xScaled - prevX
    dy = yScaled - prevY
    magAcc = hypot(dx / dt, dy / dt)
    magAccClamped = MathUtil.clamp(magAcc, -RobotConstants.MAX_ACCEL, RobotConstants.MAX_ACCEL)

    val factor = if (magAcc == 0.0) 0.0 else magAccClamped / magAcc
    val dxClamped = dx * factor
    val dyClamped = dy * factor
    val xClamped = prevX + dxClamped
    val yClamped = prevY + dyClamped

    prevX = xClamped
    prevY = yClamped

    if (controller.aButtonPressed) {
      val desAngleA = MathUtil.angleModulus(0.0 + allianceCompensation.invoke())
      if (abs(desAngleA - drive.heading.radians) > 0.075 && abs(desAngleA - drive.heading.radians) < 2 * PI - 0.075) {
        atGoal = false
        rotCtrl.setpoint = desAngleA
      }
    } else if (controller.yButtonPressed) {
      val desAngleY = MathUtil.angleModulus(PI + allianceCompensation.invoke())
      if (abs(desAngleY - drive.heading.radians) > 0.075 && abs(desAngleY - drive.heading.radians) < 2 * PI - 0.075) {
        atGoal = false
        rotCtrl.setpoint = desAngleY
      }
    }

    if (atGoal) {
      rotScaled = rotRamp.calculate(
        (if (abs(controller.rightX) < RobotConstants.ROTATION_DEADBAND) .0 else -controller.rightX) *
          drive.maxRotSpeed
      )
    } else {
      rotScaled = MathUtil.clamp(
        rotCtrl.calculate(drive.heading.radians),
        -RobotConstants.ALIGN_ROT_SPEED,
        RobotConstants.ALIGN_ROT_SPEED
      )
      atGoal = rotCtrl.atSetpoint()
    }

    val vel = Translation2d(xClamped, yClamped)

    if (fieldOriented.invoke()) {
      /** Quick fix for the velocity skewing towards the direction of rotation
       * by rotating it with offset proportional to how much we are rotating
       **/
      vel.rotateBy(Rotation2d(-rotScaled * dt * skewConstant))
      drive.set(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          vel.x * directionCompensation.invoke(),
          vel.y * directionCompensation.invoke(),
          rotScaled,
          drive.heading
        )
      )
    } else {
      drive.set(
        ChassisSpeeds(
          vel.x,
          vel.y,
          rotScaled
        )
      )
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.addDoubleProperty("currX", { if (abs(controller.leftY) < RobotConstants.TRANSLATION_DEADBAND) .0 else -controller.leftY }, null)
    builder.addDoubleProperty("currY", { if (abs(controller.leftX) < RobotConstants.TRANSLATION_DEADBAND) .0 else -controller.leftX }, null)
    builder.addDoubleProperty("prevX", { prevX }, null)
    builder.addDoubleProperty("prevY", { prevY }, null)
    builder.addDoubleProperty("dx", { dx }, null)
    builder.addDoubleProperty("dy", { dy }, null)
    builder.addDoubleProperty("dt", { dt }, null)
    builder.addDoubleProperty("magAcc", { magAcc }, null)
    builder.addDoubleProperty("magAccClamped", { magAccClamped }, null)
    builder.addStringProperty("speeds", { drive.desiredSpeeds.toString() }, null)
    builder.addDoubleProperty("skew constant", { skewConstant }, { k: Double -> skewConstant = k })
  }
}
