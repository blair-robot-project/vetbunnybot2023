package frc.team449.control.holonomic

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer.getFPGATimestamp
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.team449.control.VisionEstimator
import frc.team449.system.AHRS

class SwerveSim(
  modules: List<SwerveModule>,
  ahrs: AHRS,
  maxLinearSpeed: Double,
  maxRotSpeed: Double,
  cameras: List<VisionEstimator>,
  field: Field2d
) : SwerveDrive(modules, ahrs, maxLinearSpeed, maxRotSpeed, cameras, field) {

  private var lastTime = getFPGATimestamp()

  override fun periodic() {
    val currTime = getFPGATimestamp()
    heading = heading.plus(Rotation2d(super.desiredSpeeds.omegaRadiansPerSecond * (currTime - lastTime)))
    this.lastTime = currTime

    set(super.desiredSpeeds)

    super.periodic()
  }
}
