package frc.team449.robot2023.auto

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2023.constants.field.FieldConstants
import kotlin.math.PI

object AutoUtil {
  fun transformForPos2(pathGroup: MutableList<ChoreoTrajectory>): MutableList<ChoreoTrajectory> {
    for (index in 0 until pathGroup.size) {
      for (time in pathGroup[index].objectiveTimestamps) {
        val currentMatrix = pathGroup[index].stateMap.get(time)

        val newMatrix = MatBuilder(N2.instance, N3.instance).fill(
          currentMatrix[0, 0],
          FieldConstants.fieldWidth - currentMatrix[0, 1],
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
          FieldConstants.fieldLength - currentMatrix[0, 0],
          FieldConstants.fieldWidth - currentMatrix[0, 1],
          MathUtil.angleModulus(PI + currentMatrix[0, 2]),
          -currentMatrix[1, 0],
          -currentMatrix[1, 1],
          currentMatrix[1, 2]
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
