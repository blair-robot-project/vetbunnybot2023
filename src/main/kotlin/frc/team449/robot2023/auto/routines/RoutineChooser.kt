package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.PositionChooser

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "RedExample1" to Example(robot, PositionChooser.Positions.POSITION1, true).createCommand(),
      "RedExample2" to Example(robot, PositionChooser.Positions.POSITION2, true).createCommand(),
      "BlueExample1" to Example(robot, PositionChooser.Positions.POSITION1, false).createCommand(),
      "BlueExample2" to Example(robot, PositionChooser.Positions.POSITION2, false).createCommand(),
      "Nothing" to DoNothing(robot).createCommand(),
      "RedChoreoTest1" to TwoPiece(robot, PositionChooser.Positions.POSITION1, true).createCommand(),
      "RedChoreoTest2" to TwoPiece(robot, PositionChooser.Positions.POSITION2, true).createCommand(),
      "BlueChoreoTest1" to TwoPiece(robot, PositionChooser.Positions.POSITION1, false).createCommand(),
      "BlueChoreoTest2" to TwoPiece(robot, PositionChooser.Positions.POSITION2, false).createCommand(),
    )
  }

  init {
    updateOptions(PositionChooser.Positions.POSITION1, DriverStation.getAlliance() == DriverStation.Alliance.Red)
  }

  fun updateOptions(position: PositionChooser.Positions, isRed: Boolean) {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "Nothing")

    this.addOption(
      "Example Auto",
      if (isRed) {
        when (position) {
          PositionChooser.Positions.POSITION1 -> {
            "RedExample1"
          }

          else -> {
            "RedExample2"
          }
        }
      } else {
        when (position) {
          PositionChooser.Positions.POSITION1 -> {
            "BlueExample1"
          }

          else -> {
            "BlueExample2"
          }
        }
      }
    )

    this.addOption(
      "Choreo 2 Piece Bal",
      if (isRed) {
        when (position) {
          PositionChooser.Positions.POSITION1 -> {
            "RedChoreoTest1"
          }

          else -> {
            "RedChoreoTest2"
          }
        }
      } else {
        when (position) {
          PositionChooser.Positions.POSITION1 -> {
            "BlueChoreoTest1"
          }

          else -> {
            "BlueChoreoTest2"
          }
        }
      }
    )
  }
}
