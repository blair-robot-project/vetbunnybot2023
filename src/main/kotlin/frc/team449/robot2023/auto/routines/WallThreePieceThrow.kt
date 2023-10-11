package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class WallThreePieceThrow(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine = ChoreoRoutine(
    drive = robot.drive,
    stopEventMap = hashMapOf(
      0 to AutoUtil.stowDropCone(robot),
      2 to SequentialCommandGroup(
        WaitCommand(1.0),
        InstantCommand(robot.endEffector::throwCube),
        WaitCommand(0.25),
        InstantCommand(robot.endEffector::stop),
      ),
      4 to SequentialCommandGroup(
        WaitCommand(1.0),
        InstantCommand(robot.endEffector::throwCube),
        WaitCommand(0.25),
        InstantCommand(robot.endEffector::stop),
      )
    ),
    parallelEventMap = hashMapOf(
      0 to AutoUtil.deployCube(robot),
      1 to ArmFollower(robot.arm) { ArmPaths.cubeThrow }.alongWith(
        robot.groundIntake.retract()
      ),
      2 to SequentialCommandGroup(
        robot.groundIntake.deploy(),
        robot.groundIntake.intakeCube(),
        InstantCommand(robot.endEffector::intake),
        InstantCommand(robot.endEffector::pistonRev),
        ArmFollower(robot.arm) { ArmPaths.throwCube }
      ),
      3 to ArmFollower(robot.arm) { ArmPaths.cubeThrow }.alongWith(
        robot.groundIntake.retract()
      ),
    )
  )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) AutoUtil.transformForRed(
      ChoreoTrajectory.createTrajectory("ConeCubeThrow")
    ) else ChoreoTrajectory.createTrajectory("ConeCubeThrow")
}