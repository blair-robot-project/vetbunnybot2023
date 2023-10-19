package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.constants.auto.AutoConstants
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
      1 to PrintCommand("ggs we gottem (smooth three piece fully finished)")
    ),
    parallelEventMap = hashMapOf(
      0 to SequentialCommandGroup(
        robot.groundIntake.deploy(),
        robot.groundIntake.intakeCube(),
        InstantCommand(robot.endEffector::intake),
        InstantCommand(robot.endEffector::pistonRev),
        ArmFollower(robot.arm) { ArmPaths.autoHighCube },
        WaitCommand(3.40 - ArmPaths.autoHighCube.totalTime),
        AutoUtil.retractAndHigh(robot),
        WaitCommand(5.715 - ArmPaths.cubeAutoHigh.totalTime - 3.40),
        AutoUtil.dropCube(robot),
        InstantCommand(robot.endEffector::intake),
        InstantCommand(robot.endEffector::pistonRev),
        ArmFollower(robot.arm) { ArmPaths.autoHighCube },
        WaitCommand(7.71 - 5.715
          - AutoConstants.CUBE_DROP_WAIT_AFTER - ArmPaths.highCube.totalTime),
        robot.groundIntake.deploy(),
        robot.groundIntake.intakeCube(),
        WaitCommand(9.60 - 7.71),
        AutoUtil.retractGroundIntake(robot),
        ArmFollower(robot.arm) { ArmPaths.cubeAutoMid },
        AutoUtil.dropCube(robot)
      )
    )
  )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) AutoUtil.transformForRed(
      ChoreoTrajectory.createTrajectory("ConeCubeCubev2")
    ) else ChoreoTrajectory.createTrajectory("ConeCubeCubev2")
}
