package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil

class OnePieceTaxi(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to robot.intake.extend().andThen(
          robot.elevator.high()
        ),
        1 to robot.elevator.stow()
      ),
      stopEventMap = hashMapOf(
        1 to SequentialCommandGroup(
          WaitCommand(1.0),
          robot.manipulator.outtake(),
          WaitCommand(1.0),
          robot.manipulator.stop()
        )
      )
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("1PieceTaxi")
      )
    } else {
      ChoreoTrajectory.createTrajectory("1PieceTaxi")
    }
}
