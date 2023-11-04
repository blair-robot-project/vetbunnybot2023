package frc.team449.control

import edu.wpi.first.math.trajectory.TrapezoidProfile.State
import kotlin.math.*

class TrapezoidExponentialProfile(
  private val vMax: Double,
  private val vLim: Double,
  private val aLim: Double,
  private val aStop: Double,
  private val xGoal: Double,
  private val state: State,
  private val dt: Double = 0.02
){
  private var xFinal = xGoal - state.position
  private var stageOneTime = vLim / aLim
  private var stageTwoTime = vMax / aLim
  private var stageThreeTime = kotlin.math.sqrt((2 * xFinal) / (aLim + aLim.pow(2) / aStop))


  fun calculate(t: Double){
    var output = state
    var t1 = (t).coerceAtMost(stageOneTime)
    var t2 = (t - t1).coerceAtMost(stageTwoTime)
    var t3 = (t - t1 - t2).coerceAtMost(stageThreeTime)
    var t4 = (t - t1 - t2 - t3)
    if (t < stageOneTime) {
      output.position += 1/2 * aLim * t1.pow(2)
      output.velocity += aLim * t1
    } else if (t < stageTwoTime) {
      output.position +=
        1/2 * aLim * t1.pow(2) + t2 * vMax +
          (vMax - vLim).pow(2) / aLim * (exp(-aLim / (vMax - vLim) * t2) - 1)
      output.velocity +=
        vMax - (vMax-vLim) * exp(-aLim / (vMax - vLim) * t2)
    } else if (t < stageThreeTime) {
      output.position +=
        1/2 * aLim * t1.pow(2) + t2 * vMax +
          (vMax - vLim).pow(2) / aLim * (exp(-aLim / (vMax - vLim) * t2) - 1) +
          vMax * t3
      output.velocity += vMax
    } else {
      output.position +=
        1/2 * aLim * t1.pow(2) + t2 * vMax +
          (vMax - vLim).pow(2) / aLim * (exp(-aLim / (vMax - vLim) * t2) - 1) +
          vMax * t3 +
          vMax * t4 - 1/2 * aStop * t4.pow(2)
      output.velocity= vMax - aStop * t4
    }
  }


}