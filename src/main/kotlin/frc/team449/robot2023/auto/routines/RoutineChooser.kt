package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

    fun routineMap(): HashMap<String, Command> {
        return hashMapOf(
            "DoNothing" to DoNothing(robot).createCommand(),
            "BlueOnePiecePick" to OnePiecePick(robot, false).createCommand(),
            "RedOnePiecePick" to OnePiecePick(robot, true).createCommand()
        )
    }

    init {
        updateOptions(true)
    }

    fun updateOptions(isRed: Boolean) {
        /** Add auto options here */
        this.setDefaultOption("Do Nothing", "DoNothing")

        this.addOption(
            "One Piece and Pick",
            if (isRed) {
                "RedOnePiecePick"
            } else {
                "BlueOnePiecePick"
            }
        )
    }
}
