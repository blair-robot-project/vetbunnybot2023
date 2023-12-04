package frc.team449.robot2023.subsystems.light

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.LightConstants

/**
 * Controls an LED strip.
 * @param port The PWM port of the LED strip.
 * @param length The length of the LED strip.
 */

class Light(
  port: Int,
  length: Int
) : SubsystemBase() {

  private val lightStrip = AddressableLED(port)
  val buffer = AddressableLEDBuffer(length)

  init {
    lightStrip.setLength(buffer.length)
    lightStrip.setData(buffer)
    lightStrip.start()
  }

  override fun periodic() {
    lightStrip.setData(buffer)
  }

  companion object {
    /** Create an LED strip controller using [LightConstants]. */
    fun createLight(): Light {
      return Light(
        LightConstants.LIGHT_PORT,
        LightConstants.LIGHT_LENGTH
      )
    }
  }
}
