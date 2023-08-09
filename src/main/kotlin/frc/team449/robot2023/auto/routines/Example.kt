package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.PPRoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser

class Example(
  robot: Robot,
  position: PositionChooser.Positions,
  isRed: Boolean
) : PPRoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        /** Add event markers here */
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.Positions.POSITION1) {
      if (isRed) AutoUtil.transformForAlliancePP(Paths.POS1.EXAMPLE) { true } else Paths.POS1.EXAMPLE
    } else {
      if (isRed) AutoUtil.transformForAlliancePP(Paths.POS2.EXAMPLE) { true } else Paths.POS2.EXAMPLE
    }
}
