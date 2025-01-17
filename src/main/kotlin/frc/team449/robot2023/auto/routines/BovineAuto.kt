package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil

class BovineAuto(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        1 to robot.intake.extend().andThen(
          robot.elevator.high()
        )
      ),
      stopEventMap = hashMapOf(
        1 to WaitCommand(4.5),
        2 to SequentialCommandGroup(
          WaitCommand(1.35),
          robot.manipulator.outtake(),
          WaitCommand(1.5),
          robot.manipulator.stop(),
          robot.elevator.stow()
        )
      )
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory("686Auto")
      )
    } else {
      ChoreoTrajectory.createTrajectory("686Auto")
    }
}
