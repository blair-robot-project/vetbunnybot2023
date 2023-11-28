package frc.team449.robot2023.commands.light

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.robot2023.subsystems.light.Light

class PickupBlink {
    fun blinkGreen(light: Light): Command {
        // TODO: Finish this up and test it ig
        val cmdGroup = SequentialCommandGroup()

        cmdGroup.addRequirements(light)

        for (x in 0 until 5) {
            cmdGroup.addCommands(InstantCommand({ setColor(light, 0, 255, 0) }))
            cmdGroup.addCommands(WaitCommand(0.1))
            cmdGroup.addCommands(InstantCommand({ setColor(light, 0, 0, 0) }))
            cmdGroup.addCommands(WaitCommand(0.1))
        }

        return cmdGroup
    }

    private fun setColor(led: Light, r: Int, g: Int, b: Int) {
        for (i in 0 until led.buffer.length) {
            led.buffer.setRGB(i, r, g, b)
        }
    }
}
