package frc.team449.robot2023.auto

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj2.command.*
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2023.Robot
import frc.team449.robot2023.constants.auto.AutoConstants
import frc.team449.robot2023.constants.field.FieldConstants
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import frc.team449.robot2023.subsystems.arm.control.ArmState
import kotlin.math.PI

object AutoUtil {
  fun transformForPos2(pathGroup: MutableList<ChoreoTrajectory>): MutableList<ChoreoTrajectory> {
    for (index in 0 until pathGroup.size) {
      for (time in pathGroup[index].objectiveTimestamps) {
        val currentMatrix = pathGroup[index].stateMap.get(time)

        val newMatrix = MatBuilder(N2.instance, N3.instance).fill(
          currentMatrix[0, 0],
          FieldConstants.fieldWidth - currentMatrix[0, 1],
          -currentMatrix[0, 2],
          currentMatrix[1, 0],
          -currentMatrix[1, 1],
          -currentMatrix[1, 2]
        )

        pathGroup[index].stateMap.put(time, newMatrix)
      }
    }

    return pathGroup
  }

  fun transformForRed(pathGroup: MutableList<ChoreoTrajectory>): MutableList<ChoreoTrajectory> {
    for (index in 0 until pathGroup.size) {
      for (time in pathGroup[index].objectiveTimestamps) {
        val currentMatrix = pathGroup[index].stateMap.get(time)

        val newMatrix = MatBuilder(N2.instance, N3.instance).fill(
          FieldConstants.fieldLength - currentMatrix[0, 0],
          currentMatrix[0, 1],
          PI - currentMatrix[0, 2],
          -currentMatrix[1, 0],
          currentMatrix[1, 1],
          -currentMatrix[1, 2]
        )

        pathGroup[index].stateMap.put(time, newMatrix)
      }
    }

    return pathGroup
  }

  /** Add other methods that return commands that do groups of actions that are done
   * across different auto routines. For Charged UP, these methods were things such as
   * dropping a cone/cube, or getting in ground intake position, etc.
   */
  fun dropCone(robot: Robot): Command {
    return SequentialCommandGroup(
      RepeatCommand(
        InstantCommand(
          {
            val currState = robot.arm.desiredState.copy()
            robot.arm.moveToState(
              ArmState(
                currState.theta,
                currState.beta + Rotation2d.fromDegrees(AutoConstants.CONE_DROP_SWEEP_SPEED),
                currState.thetaVel,
                currState.betaVel
              )
            )
          }
        )
      ).withTimeout(AutoConstants.CONE_DROP_SWEEP_TIME),
      InstantCommand(robot.endEffector::pistonRev)
    )
  }

  fun dropCube(robot: Robot): Command {
    return SequentialCommandGroup(
      WaitCommand(AutoConstants.CUBE_DROP_WAIT_BEFORE),
      InstantCommand(robot.endEffector::autoReverse),
      WaitCommand(AutoConstants.CUBE_DROP_WAIT_AFTER)
    )
  }

  fun stowDropCube(robot: Robot): Command {
    return InstantCommand(robot.endEffector::holdIntake).andThen(
      ArmFollower(robot.arm) { ArmPaths.stowHigh }.andThen(dropCube(robot))
    )
  }

  fun stowDropCone(robot: Robot): Command {
    return InstantCommand(robot.endEffector::holdIntake).andThen(
      ArmFollower(robot.arm) { ArmPaths.stowHigh }.andThen(dropCone(robot))
    )
  }

  fun deployCube(robot: Robot): Command {
    return SequentialCommandGroup(
      robot.groundIntake.deploy(),
      robot.groundIntake.intakeCube(),
      InstantCommand(robot.endEffector::intake),
      InstantCommand(robot.endEffector::pistonRev),
      ArmFollower(robot.arm) { ArmPaths.highCube }
    )
  }

  fun deployCone(robot: Robot): Command {
    return SequentialCommandGroup(
      InstantCommand(robot.endEffector::intake),
      InstantCommand(robot.endEffector::pistonOn),
      ArmFollower(robot.arm) { ArmPaths.highCone }
    )
  }

  fun stowArm(robot: Robot): Command {
    return ArmFollower(robot.arm) { ArmPaths.highStow }
  }

  fun retractAndStow(robot: Robot): Command {
    return SequentialCommandGroup(
      retractGroundIntake(robot),
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) }
    )
  }

  fun retractAndHigh(robot: Robot): Command {
    return SequentialCommandGroup(
      retractGroundIntake(robot),
      ArmFollower(robot.arm) { ArmPaths.cubeHigh },
    )
  }

  fun retractAndMid(robot: Robot): Command {
    return SequentialCommandGroup(
      retractGroundIntake(robot),
      ArmFollower(robot.arm) { ArmPaths.cubeMid }
    )
  }

  fun retractGroundIntake(robot: Robot): Command {
    return SequentialCommandGroup(
      InstantCommand(robot.endEffector::strongHoldIntake),
      robot.groundIntake.retract(),
      robot.groundIntake.runOnce(robot.groundIntake::stop)
    )
  }
}
