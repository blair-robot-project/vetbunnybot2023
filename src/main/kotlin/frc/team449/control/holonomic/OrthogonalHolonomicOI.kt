package frc.team449.control.holonomic

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import frc.team449.control.OI
import frc.team449.robot2023.constants.RobotConstants
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot

/** Creates an OI to control a holonomic drivetrain.
 * The X/Y axes of one stick are used to control X/Y velocity (m/s).
 * The X axis of the other stick is used to control angular velocity (m/s).
 * This orthogonal OI will also map the A, X, B, and Y buttons on a joystick to PI/2 * k setpoints.
 * @param drive The drivetrain that the OI is controlling.
 * @param xThrottle The Y axis of the strafing joystick.
 * @param yThrottle The X axis of the strafing joystick.
 * @param rotThrottle The X axis of the rotating joystick.
 * @param rotRamp Used to ramp angular velocity.
 * @param maxAccel Max accel. Used for ramping.
 * @param fieldOriented Whether the OI X and Y translation should
 * be relative to the field rather than relative to the robot. This better be true. We are not 1727.
 */
class OrthogonalHolonomicOI(
  private val drive: HolonomicDrive,
  private val xThrottle: DoubleSupplier,
  private val yThrottle: DoubleSupplier,
  private val rotThrottle: DoubleSupplier,
  private val rotRamp: SlewRateLimiter,
  private val maxAccel: Double,
  private val fieldOriented: () -> Boolean,
  private val controller: ProfiledPIDController,
  private val yButton: BooleanSupplier,
  private val xButton: BooleanSupplier,
  private val aButton: BooleanSupplier,
  private val bButton: BooleanSupplier
) : OI, Sendable {

  init {
    controller.enableContinuousInput(-PI, PI)
    controller.setTolerance(0.0075) // ~0.429 degrees of tolerance on each side
    controller.goal = TrapezoidProfile.State(drive.heading.radians, 0.0)
  }

  /** Previous x velocity (scaled and clamped) */
  private var prevX = 0.0

  /** Previous y velocity (scaled and clamped) */
  private var prevY = 0.0

  private var prevTime = Double.NaN

  private var dx = 0.0
  private var dy = 0.0
  private var magAcc = 0.0
  private var dt = 0.0
  private var magAccClamped = 0.0

  private var rotScaled = 0.0
  private val allianceCompensation = { if (RobotConstants.ALLIANCE_COLOR == DriverStation.Alliance.Red) 0.0 else PI }
  private val directionCompensation = { if (RobotConstants.ALLIANCE_COLOR == DriverStation.Alliance.Red) -1.0 else 1.0 }

  private var atGoal = true

  /**
   * @return The [ChassisSpeeds] for the given x, y and
   * rotation input from the joystick */
  override fun get(): ChassisSpeeds {
    val currTime = Timer.getFPGATimestamp()
    if (this.prevTime.isNaN()) {
      this.prevTime = currTime - 0.02
    }
    this.dt = currTime - prevTime
    this.prevTime = currTime

    val xScaled = xThrottle.asDouble * drive.maxLinearSpeed
    val yScaled = yThrottle.asDouble * drive.maxLinearSpeed

    // Clamp the acceleration
    this.dx = xScaled - this.prevX
    this.dy = yScaled - this.prevY
    this.magAcc = hypot(dx / dt, dy / dt)
    this.magAccClamped = MathUtil.clamp(magAcc, -this.maxAccel, this.maxAccel)

    // Scale the change in x and y the same as the acceleration
    val factor = if (magAcc == 0.0) 0.0 else magAccClamped / magAcc
    val dxClamped = dx * factor
    val dyClamped = dy * factor
    val xClamped = prevX + dxClamped
    val yClamped = prevY + dyClamped

    this.prevX = xClamped
    this.prevY = yClamped

    /** Based on which button was pressed, give in the setpoint to the PID controller. */
    if (aButton.asBoolean) {
      atGoal = false
      val desAngleA = MathUtil.angleModulus(0.0 + allianceCompensation.invoke())
      if (abs(desAngleA - drive.heading.radians) > 0.075) {
        controller.goal = TrapezoidProfile.State(desAngleA, 0.0)
      }
    }
//    else if (bButton.asBoolean) {
//      atGoal = false
//      controller.goal = TrapezoidProfile.State(MathUtil.angleModulus(PI / 2 + allianceCompensation.invoke()), 0.0)
//    }
    else if (yButton.asBoolean) {
      atGoal = false
      val desAngleY = MathUtil.angleModulus(PI + allianceCompensation.invoke())
      if (abs(desAngleY - drive.heading.radians) > 0.075) {
        println("here")
        println(desAngleY)
        controller.goal = TrapezoidProfile.State(desAngleY, 0.0)
      }
    }
//    else if (xButton.asBoolean) {
//      atGoal = false
//      controller.goal = TrapezoidProfile.State(MathUtil.angleModulus(3 * PI / 2 + allianceCompensation.invoke()), 0.0)
//    }

    /** If the PID controller is at its setpoint, then allow the driver to control rotation,
     * otherwise let the PID do its thing. */
    if (atGoal) {
      rotScaled = rotRamp.calculate(rotThrottle.asDouble * drive.maxRotSpeed)
    } else {
      rotScaled = controller.calculate(drive.heading.radians) * drive.maxRotSpeed
      atGoal = controller.atGoal()
    }

    // translation velocity vector
    val vel = Translation2d(xClamped, yClamped)

    return if (this.fieldOriented()) {
      /** Quick fix for the velocity skewing towards the direction of rotation
       * by rotating it with offset proportional to how much we are rotating
       **/
      vel.rotateBy(Rotation2d(-rotScaled * dt / 2))
      ChassisSpeeds.fromFieldRelativeSpeeds(
        vel.x * directionCompensation.invoke(),
        vel.y * directionCompensation.invoke(),
        rotScaled,
        drive.heading
      )
    } else {
      ChassisSpeeds(
        vel.x,
        vel.y,
        rotScaled
      )
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.addDoubleProperty("currX", this.xThrottle::getAsDouble, null)
    builder.addDoubleProperty("currY", this.yThrottle::getAsDouble, null)
    builder.addDoubleProperty("prevX", { this.prevX }, null)
    builder.addDoubleProperty("prevY", { this.prevY }, null)
    builder.addDoubleProperty("dx", { this.dx }, null)
    builder.addDoubleProperty("dy", { this.dy }, null)
    builder.addDoubleProperty("dt", { this.dt }, null)
    builder.addDoubleProperty("magAcc", { this.magAcc }, null)
    builder.addDoubleProperty("magAccClamped", { this.magAccClamped }, null)
    builder.addStringProperty("speeds", { this.get().toString() }, null)
  }

  companion object {
    fun createOrthogonalHolonomicOI(drive: HolonomicDrive, driveController: XboxController): OrthogonalHolonomicOI {
      return OrthogonalHolonomicOI(
        drive,
        { if (abs(driveController.leftY) < RobotConstants.TRANSLATION_DEADBAND) .0 else -driveController.leftY },
        { if (abs(driveController.leftX) < RobotConstants.TRANSLATION_DEADBAND) .0 else -driveController.leftX },
        { if (abs(driveController.rightX) < RobotConstants.ROTATION_DEADBAND) .0 else -driveController.rightX },
        SlewRateLimiter(RobotConstants.RATE_LIMIT),
        RobotConstants.MAX_ACCEL,
        { driveController.leftTriggerAxis < 0.8 },
        RobotConstants.ORTHOGONAL_CONTROLLER,
        { driveController.yButtonPressed },
        { driveController.xButtonPressed },
        { driveController.aButtonPressed },
        { driveController.bButtonPressed }
      )
    }
  }
}
