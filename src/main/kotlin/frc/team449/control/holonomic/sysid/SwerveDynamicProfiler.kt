package frc.team449.control.holonomic.sysid

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.util.datalog.DoubleLogEntry
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.holonomic.SwerveDrive

/**
 * Before using this set kS, kA, kP, kI, kD = 0 and kV = 1
 */
class SwerveDynamicProfiler(
  private val drive: SwerveDrive
) : Command() {
  val timer = Timer()

  val log = DataLogManager.getLog()
  val time = DoubleLogEntry(log, "time")
  val frontLeftVol = DoubleLogEntry(log, "front left voltage")
  val frontLeftVel = DoubleLogEntry(log, "front left velocity")
  val frontLeftPos = DoubleLogEntry(log, "front left position")
  val frontRightVol = DoubleLogEntry(log, "front right voltage")
  val frontRightVel = DoubleLogEntry(log, "front right velocity")
  val frontRightPos = DoubleLogEntry(log, "front right position")
  val backRightVol = DoubleLogEntry(log, "back right voltage")
  val backRightVel = DoubleLogEntry(log, "back right velocity")
  val backRightPos = DoubleLogEntry(log, "back right position")
  val backLeftVol = DoubleLogEntry(log, "back left voltage")
  val backLeftVel = DoubleLogEntry(log, "back left velocity")
  val backLeftPos = DoubleLogEntry(log, "back left position")

  init {
    addRequirements(drive)
    timer.restart()

    DataLogManager.start()
  }

  override fun execute() {
    val v_goal = 2.0 * timer.get() / 5
    drive.set(ChassisSpeeds(v_goal, 0.0, 0.0))
    val positions = drive.getPositions()
    val velocities = drive.getStates()
    val voltages = drive.getVoltages()
    if (v_goal > 0) {
      time.append(timer.get())
      frontLeftVol.append(voltages[0])
      frontRightVol.append(voltages[1])
      backRightVol.append(voltages[2])
      backLeftVol.append(voltages[3])
      frontLeftPos.append(positions[0].distanceMeters)
      frontRightPos.append(positions[1].distanceMeters)
      backRightPos.append(positions[2].distanceMeters)
      backLeftPos.append(positions[3].distanceMeters)
      frontLeftVel.append(velocities[0].speedMetersPerSecond)
      frontRightVel.append(velocities[1].speedMetersPerSecond)
      backRightVel.append(velocities[2].speedMetersPerSecond)
      backLeftVel.append(velocities[3].speedMetersPerSecond)
    }
  }
}
