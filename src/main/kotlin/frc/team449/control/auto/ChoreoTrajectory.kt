package frc.team449.control.auto

import edu.wpi.first.math.InterpolatingMatrixTreeMap
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Filesystem
import org.json.simple.JSONArray
import org.json.simple.JSONObject
import org.json.simple.parser.JSONParser
import java.io.File
import java.io.FileReader


class ChoreoTrajectory(
  val name: String,
  val stateMap: InterpolatingMatrixTreeMap<Double, N2, N3>,
  val totalTime: Double,
  val objectiveTimestamps: ArrayList<Double>
) {

  fun initialPose(): Pose2d {
    val initialState = stateMap.get(0.0)

    return Pose2d(
      initialState[0, 0],
      initialState[0, 1],
      Rotation2d(initialState[0, 2])
    )
  }

  fun sample(t: Double): ChoreoState {
    val timeSeconds = MathUtil.clamp(t, 0.0, totalTime)
    val interpolatedMat = stateMap.get(timeSeconds)

    return ChoreoState(
      interpolatedMat[0, 0],
      interpolatedMat[0, 1],
      interpolatedMat[0, 2],
      interpolatedMat[1, 0],
      interpolatedMat[1, 1],
      interpolatedMat[1, 2]
    )
  }

  class ChoreoState(
    val xPos: Double,
    val yPos: Double,
    val theta: Double,
    val xVel: Double,
    val yVel: Double,
    val thetaVel: Double
  )

  companion object {
    fun createTrajectory(
      filename: String,
      trajName: String = filename
    ): ChoreoTrajectory {
      val path = Filesystem.getDeployDirectory().absolutePath.plus("/trajectories/$filename.json")
      val trajectory = JSONParser().parse(FileReader(File(path).absolutePath)) as JSONArray

      val last = trajectory.last() as JSONObject
      val totalTime = last["timestamp"] as Double

      val parseResults = parse(trajectory)

      return ChoreoTrajectory(
        trajName,
        parseResults.first,
        totalTime,
        parseResults.second
      )
    }

    fun createTrajectoryGroup(folderName: String): MutableList<ChoreoTrajectory> {
      val trajectoryList = mutableListOf<ChoreoTrajectory>()

      File(Filesystem.getDeployDirectory().absolutePath.plus("/trajectories/$folderName")).walk().forEach {
        if (!it.name.equals(folderName)) {
          trajectoryList.add(createTrajectory("$folderName/" + it.name.dropLast(5), folderName + it.name))
        }
      }

      return trajectoryList
    }

    private fun parse(trajectory: JSONArray): Pair<InterpolatingMatrixTreeMap<Double, N2, N3>, ArrayList<Double>> {
      val stateMap = InterpolatingMatrixTreeMap<Double, N2, N3>()

      val timestamps = arrayListOf<Double>()

      trajectory.forEach { state ->
        state as JSONObject
        val stateTime = state["timestamp"].toString().toDouble()

        timestamps.add(stateTime)

        val builder = MatBuilder(N2.instance, N3.instance)
        val matrix = builder.fill(
          state["x"].toString().toDouble(),
          state["y"].toString().toDouble(),
          state["heading"].toString().toDouble(),
          state["velocityX"].toString().toDouble(),
          state["velocityY"].toString().toDouble(),
          state["angularVelocity"].toString().toDouble())

        stateMap.put(stateTime, matrix)
      }

      return stateMap to timestamps
    }
  }
}