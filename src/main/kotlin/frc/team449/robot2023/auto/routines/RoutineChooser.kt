package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "DoNothing" to DoNothing(robot).createCommand(),
      "BlueOnePiecePick" to OnePiecePick(robot, false).createCommand(),
      "RedOnePiecePick" to OnePiecePick(robot, true).createCommand(),
      "BlueOnePieceTaxi" to OnePieceTaxi(robot, false).createCommand(),
      "RedOnePieceTaxi" to OnePieceTaxi(robot, true).createCommand(),
      "RedOnePieceTaxiRight" to OnePieceTaxiRight(robot, true).createCommand(),
      "BlueOnePieceTaxiRight" to OnePieceTaxiRight(robot, false).createCommand(),
      "Blue686Trust" to BovineAuto(robot, false).createCommand(),
      "Red686Trust" to BovineAuto(robot, true).createCommand()
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

    this.addOption(
      "Bovine Auto",
      if (isRed) {
        "Red686Trust"
      } else {
        "Blue686Trust"
      }
    )

    this.addOption(
      "One Piece and Taxi",
      if (isRed) {
        "RedOnePieceTaxi"
      } else {
        "BlueOnePieceTaxi"
      }
    )

    this.addOption(
      "One Piece and Taxi Right",
      if (isRed) {
        "RedOnePieceTaxiRight"
      } else {
        "BlueOnePieceTaxiRight"
      }
    )
  }
}
