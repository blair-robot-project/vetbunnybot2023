package frc.team449.control.auto

import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot

interface ChoreoRoutineStructure {

  val routine: ChoreoRoutine

  val trajectory: MutableList<ChoreoTrajectory>

  fun createCommand(robot: Robot): Command {
    return routine.createRoutine(trajectory)
  }
}
