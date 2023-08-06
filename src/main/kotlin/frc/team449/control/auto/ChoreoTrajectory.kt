package frc.team449.control.auto

import edu.wpi.first.math.InterpolatingMatrixTreeMap
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.auto.AutoConstants
import org.json.simple.JSONArray
import org.json.simple.JSONObject
import org.json.simple.parser.JSONParser
import java.io.File
import java.io.FileReader


class ChoreoTrajectory(
  val filename: String
) {

  private val path = Filesystem.getDeployDirectory().absolutePath.plus("/trajectories/$filename.json")
  private val trajectory = JSONParser().parse(FileReader(File(path).absolutePath)) as JSONArray

  private val initialState = trajectory[0] as JSONObject
  val initialPose = Pose2d(
    initialState["x"] as Double,
    initialState["y"] as Double,
    Rotation2d(initialState["heading"] as Double)
  )

  private var stateMap: InterpolatingMatrixTreeMap<Double, N2, N3> = InterpolatingMatrixTreeMap()

  var totalTime = 0.0

  init {
    parse()
  }

  private fun parse() {
    trajectory.forEach { state ->
      state as JSONObject
      val stateTime = state["timestamp"].toString().toDouble()

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

    val last = trajectory.last() as JSONObject
    totalTime = last["timestamp"] as Double
  }
  fun sample(t: Double): Matrix<N2, N3> {
    val timeSeconds = MathUtil.clamp(t, 0.0, totalTime)
    return stateMap.get(timeSeconds)
  }

  companion object {
    fun createTrajectory(filename: String): ChoreoTrajectory {
      return ChoreoTrajectory(filename)
    }
  }
}