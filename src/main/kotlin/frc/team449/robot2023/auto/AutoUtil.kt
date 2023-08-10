package frc.team449.robot2023.auto

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.DriverStation
import frc.team449.control.auto.ChoreoTrajectory
import java.util.Collections
import java.util.function.BooleanSupplier
import kotlin.math.PI

object AutoUtil {
  fun transformForPosition2PP(trajList: MutableList<PathPlannerTrajectory>): MutableList<PathPlannerTrajectory> {
    val correctedTrajList: MutableList<PathPlannerTrajectory> = MutableList(
      trajList.size
    ) { PathPlannerTrajectory() }

    Collections.copy(correctedTrajList, trajList)

    for ((index, _) in correctedTrajList.withIndex()) {
      for (s in correctedTrajList[index].states) {
        s as PathPlannerTrajectory.PathPlannerState
        s.poseMeters = Pose2d(s.poseMeters.x, AutoConstants.FIELD_WIDTH - s.poseMeters.y, -s.poseMeters.rotation)
        s.holonomicAngularVelocityRadPerSec *= -1.0
        s.holonomicRotation *= -1.0
      }
    }

    return correctedTrajList
  }

  /**
   * Note: This version of transform for alliance is for Charged Up where each alliance is mirrored across the y-axis instead of the usual diagonal
   */
  fun transformForAlliancePP(pathGroup: MutableList<PathPlannerTrajectory>, isRed: BooleanSupplier): MutableList<PathPlannerTrajectory> {
    val correctedPathGroup: MutableList<PathPlannerTrajectory> = MutableList(
      pathGroup.size
    ) { PathPlannerTrajectory() }

    Collections.copy(correctedPathGroup, pathGroup)

    if (isRed.asBoolean) {
      for ((index, _) in correctedPathGroup.withIndex()) {
        correctedPathGroup[index] = PathPlannerTrajectory.transformTrajectoryForAlliance(
          correctedPathGroup[index],
          DriverStation.getAlliance()
        )

        for (s in correctedPathGroup[index].states) {
          s as PathPlannerTrajectory.PathPlannerState
          s.poseMeters = Pose2d(
            AutoConstants.FIELD_LENGTH - s.poseMeters.x,
            AutoConstants.FIELD_WIDTH - s.poseMeters.y,
            s.poseMeters.rotation.plus(Rotation2d(PI))
          )
          s.holonomicRotation = s.holonomicRotation.plus(Rotation2d(PI))
        }
      }
    }

    return correctedPathGroup
  }

  fun transformForPos2(pathGroup: MutableList<ChoreoTrajectory>): MutableList<ChoreoTrajectory> {
    for (index in 0 until pathGroup.size) {
      for (time in pathGroup[index].objectiveTimestamps) {
        val currentMatrix = pathGroup[index].stateMap.get(time)

        val newMatrix = MatBuilder(N2.instance, N3.instance).fill(
          currentMatrix[0, 0],
          AutoConstants.FIELD_WIDTH - currentMatrix[0, 1],
          -currentMatrix[0, 2],
          currentMatrix[1, 0],
          -currentMatrix[1, 1],
          -currentMatrix[1, 2]
        )

        pathGroup[index].stateMap.put(time, newMatrix)
      }
    }

    return pathGroup
  }

  fun transformForRed(pathGroup: MutableList<ChoreoTrajectory>): MutableList<ChoreoTrajectory> {
    for (index in 0 until pathGroup.size) {
      for (time in pathGroup[index].objectiveTimestamps) {
        val currentMatrix = pathGroup[index].stateMap.get(time)

        val newMatrix = MatBuilder(N2.instance, N3.instance).fill(
          AutoConstants.FIELD_LENGTH - currentMatrix[0, 0],
          currentMatrix[0, 1],
          PI - currentMatrix[0, 2],
          -currentMatrix[1, 0],
          currentMatrix[1, 1],
          -currentMatrix[1, 2]
        )

        pathGroup[index].stateMap.put(time, newMatrix)
      }
    }

    return pathGroup
  }

  /** Add other methods that return commands that do groups of actions that are done
   * across different auto routines. For Charged UP, these methods were things such as
   * dropping a cone/cube, or getting in ground intake position, etc.
   */
}
