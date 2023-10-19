package frc.team449.robot2023.commands.autoBalance

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2023.constants.auto.AutoConstants
import kotlin.math.PI

class AutoBalance(
  private val controller: PIDController,
  private val drive: SwerveDrive,
  private val speedMetersPerSecond: Double
) : CommandBase() {

  init {
    addRequirements(drive)
  }

  override fun initialize() {
    addRequirements(drive)
    controller.enableContinuousInput(-PI, PI)
    controller.setTolerance(0.125) /* .05 rad tolerance ~3 degrees */
    controller.setpoint = 0.0
  }

  override fun execute() {
    drive.set(
      ChassisSpeeds(
        MathUtil.clamp(
          controller.calculate(drive.roll.radians),
          -speedMetersPerSecond,
          speedMetersPerSecond
        ),
        0.0,
        0.0
      )
    )
  }

  override fun isFinished(): Boolean {
    return controller.atSetpoint()
  }

  override fun end(interrupted: Boolean) {
    drive.stop()
  }

  companion object {
    fun genCmd(drive: SwerveDrive): AutoBalance {
      return AutoBalance(
        PIDController(AutoConstants.AUTO_BAL_KP, 0.0, 0.0),
        drive,
        AutoConstants.ADJUST_SPEED
      )
    }
  }
}
