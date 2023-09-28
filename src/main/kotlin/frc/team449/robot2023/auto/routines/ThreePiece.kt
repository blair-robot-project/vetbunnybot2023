package frc.team449.robot2023.auto.routines

import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

/**
 * This is an example of how to use the Choreo structure that has been created. It's also a 3 piece lol
 *  Btw this should only be run on the far side which is why position isn't a parameter
 */
class ThreePiece(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine = ChoreoRoutine(
    drive = robot.drive,
    stopEventMap = hashMapOf(
      0 to AutoUtil.stowDropCone(robot),
      2 to AutoUtil.dropCube(robot),
      4 to AutoUtil.dropCube(robot).andThen(ArmFollower(robot.arm) { ArmPaths.midBack })
    ),
    parallelEventMap = hashMapOf(
      0 to AutoUtil.deployCube(robot),
      1 to AutoUtil.retractAndHigh(robot),
      2 to AutoUtil.deployCube(robot),
      3 to AutoUtil.retractAndMid(robot),
    )
  )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) AutoUtil.transformForRed(
      ChoreoTrajectory.createTrajectory("ConeCubeCube")
    ) else ChoreoTrajectory.createTrajectory("ConeCubeCube")
}
