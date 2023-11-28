package frc.team449.control.auto

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.*
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2023.constants.auto.AutoConstants
import io.github.oblarg.oblog.annotations.Config
import kotlin.math.abs

class ChoreoRoutine(
    @field:Config.PIDController(name = "X PID") var xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
    @field:Config.PIDController(name = "Y PID") var yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
    @field:Config.PIDController(name = "Rotation PID") var thetaController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0),
    private val drive: HolonomicDrive,
    private val stopEventMap: HashMap<Int, Command> = HashMap(),
    private val parallelEventMap: HashMap<Int, Command> = HashMap(),
    private val poseTol: Pose2d = Pose2d(0.05, 0.05, Rotation2d(0.05)),
    private val resetPosition: Boolean = false,
    private val resetPositionTolerance: Pose2d = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
    private val timeout: Double = 0.35
) {

    private fun resetPose(trajectory: ChoreoTrajectory): Command {
        val poseError = drive.pose.relativeTo(trajectory.initialPose())

        if (abs(poseError.x) < resetPositionTolerance.x &&
            abs(poseError.y) < resetPositionTolerance.y &&
            abs(poseError.rotation.radians) < resetPositionTolerance.rotation.radians
        ) {
            return PrintCommand("Pose not reset.")
        }

        println(trajectory.name)

        return InstantCommand({ drive.pose = trajectory.initialPose() })
    }

    fun createRoutine(trajectories: MutableList<ChoreoTrajectory>): Command {
        val ezraGallun = SequentialCommandGroup(
            resetPose(trajectories[0]),
            stopEventMap.getOrDefault(0, InstantCommand())
        )

        for (i in 0 until trajectories.size) {
            ezraGallun.addCommands(
                ParallelCommandGroup(
                    ChoreoFollower(
                        drive,
                        trajectories[i],
                        xController,
                        yController,
                        thetaController,
                        poseTol,
                        timeout,
                        resetPosition
                    ),
                    parallelEventMap.getOrDefault(i, InstantCommand())
                )
            )
            ezraGallun.addCommands(stopEventMap.getOrDefault(i + 1, InstantCommand()))
        }

        return ezraGallun
    }
}
