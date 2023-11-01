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
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class TwoPieceBump(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine = ChoreoRoutine(
    drive = robot.drive,
    stopEventMap = hashMapOf(
      0 to AutoUtil.stowDropCone(robot),
      1 to SequentialCommandGroup(
        ArmFollower(robot.arm) { ArmPaths.stowHigh },
        WaitCommand(0.35),
        AutoUtil.dropCube(robot),
        ArmFollower(robot.arm) { ArmPaths.highBack },
        PrintCommand("insane auto (two piece bump fully finished)")
      )
    ),
    parallelEventMap = hashMapOf(
      0 to SequentialCommandGroup(
        robot.groundIntake.deploy(),
        robot.groundIntake.intakeCube(),
        InstantCommand(robot.endEffector::intake),
        InstantCommand(robot.endEffector::pistonRev),
        ArmFollower(robot.arm) { ArmPaths.autoHighCube },
        WaitCommand(5.30 - ArmPaths.autoHighCube.totalTime),
        AutoUtil.retractGroundIntake(robot),
        ArmFollower(robot.arm) { ArmPaths.cubeStow }
      )
    )
  )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) AutoUtil.transformForRed(
      ChoreoTrajectory.createTrajectory("ConeCubeBump")
    ) else ChoreoTrajectory.createTrajectory("ConeCubeBump")
}