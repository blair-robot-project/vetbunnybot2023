package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "DropCone" to DropCone(robot).createCommand(robot),
      "RedFarThreePiece" to ThreePiece(robot, true).createCommand(robot),
      "BlueFarThreePiece" to ThreePiece(robot, false).createCommand(robot),
      "BlueThrowThreePiece" to WallThreePieceThrow(robot, false).createCommand(robot),
      "RedThrowThreePiece" to WallThreePieceThrow(robot, true).createCommand(robot),
      "BlueTwoPieceBal" to TwoPieceBal(robot, false).createCommand(robot),
      "RedTwoPieceBal" to TwoPieceBal(robot, true).createCommand(robot),
      "BlueCubeBal" to CubeBalance(robot, false).createCommand(robot),
      "RedCubeBal" to CubeBalance(robot, true).createCommand(robot),
      "BlueConeCubeBump" to TwoPieceBump(robot, false).createCommand(robot),
      "RedConeCubeBump" to TwoPieceBump(robot, true).createCommand(robot)
    )
  }

  init {
    updateOptions(DriverStation.getAlliance() == DriverStation.Alliance.Red)
  }

  fun updateOptions(isRed: Boolean) {
    /** Add auto options here */
    this.setDefaultOption("Drop Cone", "DropCone")

    this.addOption(
      "Smooth Three Piece",
      if (isRed) "RedFarThreePiece"
      else "BlueFarThreePiece"
    )

    this.addOption(
      "Bump Three Piece Throw",
      if (isRed) "RedThrowThreePiece"
      else "BlueThrowThreePiece"
    )

    this.addOption(
      "Smooth Two Piece Bal",
      if (isRed) "RedTwoPieceBal"
      else "BlueTwoPieceBal"
    )

    this.addOption(
      "Center Cube Balance",
      if (isRed) "RedCubeBal"
      else "BlueCubeBal"
    )

    this.addOption(
      "Bump Two Piece",
      if (isRed) "RedConeCubeBump"
      else "BlueConeCubeBump"
    )
  }
}
