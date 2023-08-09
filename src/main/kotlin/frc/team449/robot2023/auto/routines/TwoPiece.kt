package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.PositionChooser

/**
 * This is an example of how to use the Choreo structure that has been created. It's also a 3 piece lol
 */
class TwoPiece(
  robot: Robot,
  position: PositionChooser.Positions,
  isRed: Boolean
): ChoreoRoutineStructure {

  override val routine = ChoreoRoutine(
    drive = robot.drive,
    stopEventMap = hashMapOf(
      0 to PrintCommand("Theoretically scoring cone high"),
      2 to PrintCommand("Theoretically scoring cube high"),
      4 to PrintCommand("Theoretically scoring cube mid")
    ),
    parallelEventMap = hashMapOf(
      0 to hashMapOf(0.0 to PrintCommand("Theoretically moving to cube pickup and extending ground intake")),
      1 to hashMapOf(0.0 to PrintCommand("Theoretically retracting ground intake and moving to high")),
      2 to hashMapOf(0.0 to PrintCommand("Theoretically moving to cube pickup and extending ground intake")),
      3 to hashMapOf(0.0 to PrintCommand("Theoretically retracting ground intake and moving to mid"))
    )
  )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (position == PositionChooser.Positions.POSITION1) {
      if (isRed) AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("ConeCubeCube")
      ) else ChoreoTrajectory.createTrajectory("ConeCubeCube")
    } else {
      if (isRed) AutoUtil.transformForPos2(
        AutoUtil.transformForRed(
          ChoreoTrajectory.createTrajectory("ConeCubeCube")
        )
      ) else AutoUtil.transformForPos2(
        ChoreoTrajectory.createTrajectory("ConeCubeCube")
      )
    }

}