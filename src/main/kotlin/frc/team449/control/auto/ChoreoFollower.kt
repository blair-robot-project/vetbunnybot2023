package frc.team449.control.auto


import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2023.auto.AutoConstants
import java.time.Instant

class ChoreoFollower(
  private val drivetrain: HolonomicDrive,
  private val trajectory: ChoreoTrajectory,
  private val parallelEvents: HashMap<Double, Command>,
  private val xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
  private val yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
  private val thetaController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0),
  poseTol: Pose2d = Pose2d(0.05, 0.05, Rotation2d(0.05)),
  private val timeout: Double = 0.65,
  private val resetPose: Boolean = false
): CommandBase() {

  private val timer = Timer()

  init {
    addRequirements(drivetrain)

    xController.reset()
    xController.setTolerance(poseTol.x)

    yController.reset()
    yController.setTolerance(poseTol.y)

    thetaController.reset()
    thetaController.setTolerance(poseTol.rotation.radians)

    if (resetPose) {
      drivetrain.pose = trajectory.initialPose
    }
  }

  private fun calculate(currPose: Pose2d, desiredMatrix: Matrix<N2, N3>): ChassisSpeeds {
    val xFF = desiredMatrix[1, 0]
    val yFF = desiredMatrix[1, 1]
    val angFF = desiredMatrix[1, 2]

    val xPID = xController.calculate(currPose.x, desiredMatrix[0, 0])
    val yPID = yController.calculate(currPose.y, desiredMatrix[0, 1])
    val angPID = thetaController.calculate(currPose.rotation.radians, desiredMatrix[0, 2])

    return ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xPID, yFF + yPID, angFF + angPID, currPose.rotation)
  }

  private fun parallelEventPop() {
    parallelEvents.remove(parallelEvents.keys.first())
  }

  private fun allControllersAtReference(): Boolean {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()
  }

  override fun initialize() {
    xController.reset()
    yController.reset()
    thetaController.reset()

    timer.restart()
  }

  override fun execute() {
    val eventTime = timer.get()

    for (event in parallelEvents) {
      if (eventTime >= event.key) {

        event.value.schedule()

        parallelEventPop()
      }
    }

    val currTime = timer.get()

    val desiredMatrix = trajectory.sample(currTime)

    drivetrain.set(calculate(drivetrain.pose, desiredMatrix))
  }

  override fun isFinished(): Boolean {
    return (timer.hasElapsed(trajectory.totalTime) && allControllersAtReference()) || (timer.hasElapsed(trajectory.totalTime + timeout))
  }

  override fun end(interrupted: Boolean) {
    timer.stop()
    timer.reset()

    println("${trajectory.filename} auto completed.")
    drivetrain.stop()
  }
}