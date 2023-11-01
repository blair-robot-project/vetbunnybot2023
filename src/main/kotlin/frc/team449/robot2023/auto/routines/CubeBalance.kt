package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.commands.autoBalance.AutoBalance
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class CubeBalance(
  robot: Robot,
  isRed: Boolean
): ChoreoRoutineStructure {
  override val routine = ChoreoRoutine(
    drive = robot.drive,
    stopEventMap = hashMapOf(
      0 to AutoUtil.stowDropCube(robot),
      1 to AutoBalance.genCmd(robot.drive)
    ),
    parallelEventMap = hashMapOf(
      0 to SequentialCommandGroup(
        ArmFollower(robot.arm) { ArmPaths.highStow },

        InstantCommand(robot.endEffector::stop),
        InstantCommand(robot.endEffector::pistonOn)
      )
    )
  )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) AutoUtil.transformForRed(
      ChoreoTrajectory.createTrajectory("CubeBal")
    ) else ChoreoTrajectory.createTrajectory("CubeBal")
}