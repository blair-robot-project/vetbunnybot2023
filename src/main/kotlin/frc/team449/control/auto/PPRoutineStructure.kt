package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.wpilibj2.command.Command

interface PPRoutineStructure {

  val routine: HolonomicRoutine

  val trajectory: MutableList<PathPlannerTrajectory>

  fun createCommand(): Command {
    return routine.createRoutine(trajectory)
  }
}
