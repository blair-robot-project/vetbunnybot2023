package frc.team449.robot2023.auto.routines

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import frc.team449.robot2023.subsystems.arm.control.ArmState

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
      4 to AutoUtil.dropCube(robot).andThen(
        ArmFollower(robot.arm) { ArmPaths.midBack }
      )
    ),
    parallelEventMap = hashMapOf(
      0 to AutoUtil.deployCube(robot),
      1 to ArmFollower(robot.arm) { ArmPaths.cubeHigh }.alongWith(
        robot.groundIntake.retract()
      ),
      2 to SequentialCommandGroup(
        InstantCommand(robot.endEffector::intake),
        InstantCommand(robot.endEffector::pistonRev),
        ArmFollower(robot.arm) { ArmPaths.highCube },
        WaitUntilCommand { robot.drive.pose.y < 11.65 },
        robot.groundIntake.deploy(),
        robot.groundIntake.intakeCube()
      ),
      3 to ParallelCommandGroup(
        ArmFollower(robot.arm) { ArmPaths.cubeMid }.andThen(
          InstantCommand({
            robot.arm.moveToState(
              ArmState(
                Rotation2d.fromDegrees(80.0),
                Rotation2d.fromDegrees(110.0)
              )
            )
          })
        ),
        robot.groundIntake.retract()
      )
    )
  )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) AutoUtil.transformForRed(
      ChoreoTrajectory.createTrajectory("ConeCubeCube")
    ) else ChoreoTrajectory.createTrajectory("ConeCubeCube")
}
