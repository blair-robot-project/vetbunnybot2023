package frc.team449.robot2023.commands.light

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot
import frc.team449.robot2023.constants.subsystem.ElevatorConstants
import frc.team449.robot2023.constants.subsystem.LightConstants
import kotlin.math.ceil

class ElevatorMoving(
  private val robot: Robot
) : Command() {

  init {
    addRequirements(robot.light)
  }

  override fun runsWhenDisabled(): Boolean {
    return true
  }

  private var firstHue = 0.0

  private val desireds = mutableListOf<Double>()

  override fun execute() {
    desireds.add(robot.elevator.desiredState.first)
    val currentPct = MathUtil.clamp(desireds.last() / ElevatorConstants.HIGH_DISTANCE, 0.0, 1.0)

    val whiteEnd1 = (LightConstants.SECTION_1_END - ceil(currentPct * (LightConstants.SECTION_1_END - LightConstants.SECTION_1_START))).toInt()
    val whiteEnd2 = (LightConstants.SECTION_2_END - ceil(currentPct * (LightConstants.SECTION_2_END - LightConstants.SECTION_2_START))).toInt()

    for (i in LightConstants.SECTION_1_END downTo whiteEnd1) {
      robot.light.setRGB(i, 0, 0, 255)
//      if (LightConstants.SECTION_1_END == whiteEnd1) continue
//      val hue = MathUtil.inputModulus(firstHue + i * 180 / (LightConstants.SECTION_1_END - whiteEnd1), 0.0, 180.0)
//
//      robot.light.setHSV(i, hue.toInt(), 255, 255)
    }

    for (i in LightConstants.SECTION_2_END downTo whiteEnd2) {
      robot.light.setRGB(i, 0, 0, 255)
//      if (LightConstants.SECTION_2_END == whiteEnd2) continue
//      val hue = MathUtil.inputModulus(firstHue + (i - LightConstants.SECTION_2_START) * 180 / (LightConstants.SECTION_2_END - whiteEnd2), 0.0, 180.0)
//
//      robot.light.setHSV(i, hue.toInt(), 255, 255)
    }

    for (i in LightConstants.SECTION_1_START until whiteEnd1) {
      robot.light.setRGB(i, 255, 255, 255)
    }

    for (i in LightConstants.SECTION_2_START until whiteEnd2) {
      robot.light.setRGB(i, 255, 255, 255)
    }

//    firstHue += 1.5
//    firstHue = MathUtil.inputModulus(firstHue, 0.0, 180.0)
  }
}
