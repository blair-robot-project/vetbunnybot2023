package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.PositionChooser

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "DropCone" to DropCone(robot).createCommand(robot),
      "RedFarThreePiece" to ThreePiece(robot, true).createCommand(robot),
      "BlueFarThreePiece" to ThreePiece(robot, false).createCommand(robot)
    )
  }

  init {
    updateOptions(PositionChooser.Positions.CENTER, DriverStation.getAlliance() == DriverStation.Alliance.Red)
  }

  fun updateOptions(position: PositionChooser.Positions, isRed: Boolean) {
    /** Add auto options here */
    this.setDefaultOption("Drop Piece", "DropCone")

    this.addOption(
      "Three Piece Taxi Far",
      if (isRed) {
        "RedFarThreePiece"
      } else {
        "BlueFarThreePiece"
      }
    )



    // TODO: CREATE OPTIONS
  }
}
