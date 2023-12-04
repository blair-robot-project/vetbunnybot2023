package frc.team449.robot2023.commands.light

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.subsystems.light.Light

class ConeAnimation(
  private val led: Light
) : Command() {

  init {
    addRequirements(led)
  }

  private var firstIntensity = 175.0

  override fun execute() {
    for (i in 0 until led.buffer.length) {
      // This number is related to how many lights will show up between the high and low intensity
      val intensity = MathUtil.inputModulus(firstIntensity + i * 37.5 / led.buffer.length, 175.0, 255.0)
      led.buffer.setHSV(i, 30, 255, intensity.toInt())

      // The i * 255.0 relates to how fast it will cycle in between the high and low intensity
      firstIntensity += 0.075
      firstIntensity = MathUtil.inputModulus(firstIntensity, 175.0, 255.0)
    }
  }
}
