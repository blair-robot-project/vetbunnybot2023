package frc.team449.robot2023.auto

import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory

/**
 * Either:
 *   Deprecate the Paths obj structure. Instead of having mutated versions of these trajectories due to AutoUtil (unless these trajectories actually get
 *      a deep copy created), we should just call the creator function for each trajectory in the routine class.
 *
 *   Store lambda suppliers that create a new trajectory when called for in other classes
 */
object Paths {
  object POS1 {
    /** Description of path: Just an example at position 1  */
    val EXAMPLE: MutableList<PathPlannerTrajectory> =
      PathPlanner.loadPathGroup(
        "example1",
        PathPlanner.getConstraintsFromPath("example1")
      )
  }

  object POS2 {
    val EXAMPLE: MutableList<PathPlannerTrajectory> =
      AutoUtil.transformForPosition2PP(
        PathPlanner.loadPathGroup(
          "example1",
          PathPlanner.getConstraintsFromPath("example1")
        )
      )
  }
}
