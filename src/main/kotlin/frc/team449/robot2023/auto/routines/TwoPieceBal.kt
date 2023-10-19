package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.commands.autoBalance.AutoBalance
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class TwoPieceBal(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine = ChoreoRoutine(
    drive = robot.drive,
    stopEventMap = hashMapOf(
      0 to AutoUtil.stowDropCone(robot),
      2 to AutoUtil.dropCube(robot),
      3 to AutoBalance.genCmd(robot.drive)
    ),
    parallelEventMap = hashMapOf(
      0 to AutoUtil.deployCube(robot),
      1 to AutoUtil.retractAndHigh(robot),
      2 to ArmFollower(robot.arm) { ArmPaths.highStow }.alongWith(
        InstantCommand(robot.endEffector::pistonOn)
      )
    )
  )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) AutoUtil.transformForRed(
      ChoreoTrajectory.createTrajectory("ConeCubeBal")
    ) else ChoreoTrajectory.createTrajectory("ConeCubeBal")
}