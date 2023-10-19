package frc.team449.robot2023.subsystems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelIncoming
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.robot2023.Robot
import frc.team449.robot2023.commands.arm.ArmSweep
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import frc.team449.robot2023.subsystems.arm.control.ArmState
import kotlin.math.PI
import kotlin.math.abs

class ControllerBindings(
  private val driveController: XboxController,
  private val mechanismController: XboxController,
  private val robot: Robot
) : Sendable {

  private fun changeCone(): Command {
    return robot.endEffector.runOnce(robot.endEffector::pistonOn).andThen(
      ConditionalCommand(
        SequentialCommandGroup(
          robot.groundIntake.retract(),
          WaitCommand(0.5), // wait for the intake to fully retract (this might take more time)
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CONE) }
            .withInterruptBehavior(kCancelIncoming)
        ),
        InstantCommand()
      ) { robot.arm.desiredState == ArmConstants.CUBE }
    )
  }

  private fun changeCube(): Command {
    return robot.endEffector.runOnce(robot.endEffector::pistonRev).andThen(
      ConditionalCommand(
        SequentialCommandGroup(
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CUBE) }
            .withInterruptBehavior(kCancelIncoming),
          WaitUntilCommand {
            robot.arm.distanceBetweenStates(robot.arm.state, ArmConstants.CUBE) <= 0.05 &&
              robot.arm.state.betaVel <= 0.05
            robot.arm.state.thetaVel <= 0.05
          },
          robot.groundIntake.deploy()
        ),
        InstantCommand()
      ) { robot.arm.desiredState == ArmConstants.CONE }
    )
  }

  fun bindButtons() {
    // TODO: LED stuff if we get there
//    Trigger { robot.endEffector.chooserPiston.get() == DoubleSolenoid.Value.kReverse }.onTrue(
//      CubeAnimation(robot.light)
//    ).onFalse(
//      ConeAnimation(robot.light)
//    )
//
//    Trigger { !robot.infrared.get() }.onTrue(
//      PickupBlink().blinkGreen(robot)
//    )

    JoystickButton(driveController, XboxController.Button.kRightBumper.value).onTrue(
      ConditionalCommand(
        robot.endEffector.runOnce(robot.endEffector::intake),
        SequentialCommandGroup(
          robot.groundIntake.teleopCube(),
          robot.endEffector.runOnce(robot.endEffector::intake)
        )
      ) { robot.endEffector.chooserPiston.get() == DoubleSolenoid.Value.kForward }
    ).onFalse(
      SequentialCommandGroup(
        robot.groundIntake.runOnce(robot.groundIntake::stop),
        robot.endEffector.runOnce(robot.endEffector::holdIntake)
      )
    )

    Trigger { driveController.leftTriggerAxis > 0.8 }.onTrue(
      SequentialCommandGroup(
        robot.endEffector.runOnce { robot.endEffector.pistonRev() },
        ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CUBE) }
          .withInterruptBehavior(kCancelIncoming),
        WaitUntilCommand {
          robot.arm.distanceBetweenStates(robot.arm.state, ArmConstants.CUBE) <= 0.025 &&
            robot.arm.state.betaVel <= 0.025 &&
            robot.arm.state.thetaVel <= 0.025
        },
        robot.groundIntake.deploy(),
        robot.groundIntake.teleopCube()
      ).alongWith(
        RepeatCommand(InstantCommand(robot.arm::holdArm))
      )
    ).onFalse(
      SequentialCommandGroup(
        robot.groundIntake.retract(),
        robot.groundIntake.runOnce(robot.groundIntake::stop),
        ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) }
          .withInterruptBehavior(kCancelIncoming)
      )
    )

    JoystickButton(driveController, XboxController.Button.kLeftBumper.value).onTrue(
      robot.endEffector.runOnce(robot.endEffector::intakeReverse)
    ).onFalse(
      robot.endEffector.runOnce(robot.endEffector::stop).andThen(
        robot.groundIntake.runOnce(robot.groundIntake::stop)
      )
    )

    Trigger { mechanismController.pov != 1 }.onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.SINGLE) }
        .withInterruptBehavior(kCancelIncoming)
    )

    // drive speed overdrive trigger
    Trigger { driveController.rightTriggerAxis >= .75 }.onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 1.0 }).andThen(
        InstantCommand({ robot.drive.maxRotSpeed = PI / 4 })
      )
    ).onFalse(
      InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED }).andThen(
        InstantCommand({ robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED })
      )
    )

    JoystickButton(mechanismController, XboxController.Button.kRightBumper.value).onTrue(
      changeCube()
    )

    JoystickButton(mechanismController, XboxController.Button.kLeftBumper.value).onTrue(
      changeCone()
    )

    Trigger { robot.arm.desiredState == ArmConstants.STOW }.onTrue(
      robot.endEffector.runOnce(robot.endEffector::strongHoldIntake)
    ).onFalse(
      robot.endEffector.runOnce(robot.endEffector::holdIntake)
    )

    JoystickButton(mechanismController, XboxController.Button.kB.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.LOW) }
        .withInterruptBehavior(kCancelIncoming)
    )

    Trigger { mechanismController.leftTriggerAxis > 0.8 }.onTrue(
      robot.endEffector.runOnce(robot.endEffector::intakeReverse)
    ).onFalse(
      robot.endEffector.runOnce(robot.endEffector::stop).andThen(
        robot.groundIntake.runOnce(robot.groundIntake::stop)
      )
    )

    JoystickButton(mechanismController, XboxController.Button.kBack.value).onTrue(
      ArmFollower(robot.arm) {
        robot.arm.chooseTraj(ArmConstants.STOW)
      }.withInterruptBehavior(kCancelIncoming)
    )

    JoystickButton(mechanismController, XboxController.Button.kStart.value).onTrue(
      ArmFollower(robot.arm) {
        robot.arm.chooseTraj(ArmConstants.MID)
      }.withInterruptBehavior(kCancelIncoming)
    )

    JoystickButton(mechanismController, XboxController.Button.kX.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.BACK) }
        .withInterruptBehavior(kCancelIncoming).andThen(
          robot.endEffector::pistonOn
        )
    )

    JoystickButton(mechanismController, XboxController.Button.kY.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) }
        .withInterruptBehavior(kCancelIncoming)
    )

    Trigger { abs(mechanismController.rightTriggerAxis) > 0.1 }.onTrue(
      ArmSweep(
        robot.arm,
        { mechanismController.rightTriggerAxis },
        Rotation2d.fromDegrees(20.0)
      ).until { abs(mechanismController.rightTriggerAxis) < 0.1 }
    )

//    TODO: Orbit Heading Align
//    JoystickButton(driveController, XboxController.Button.kA.value).onTrue(
//      HeadingAlign(
//        robot.drive,
//        robot.oi,
//        Translation2d(),
//      ).until { driveController.aButtonReleased }
//    )

    Trigger { abs(mechanismController.leftY) > 0.3 || abs(mechanismController.rightY) > 0.3 }.onTrue(
      ParallelRaceGroup(
        WaitUntilCommand { abs(mechanismController.leftY) <= 0.3 && abs(mechanismController.rightY) <= 0.3 },
        RepeatCommand(
          robot.arm.runOnce {
            val newState = ArmState(
              Rotation2d(robot.arm.desiredState.theta.radians),
              Rotation2d(robot.arm.desiredState.beta.radians),
              0.0,
              0.0
            )
            newState.beta =
              Rotation2d(newState.beta.radians - MathUtil.applyDeadband(mechanismController.leftY, .3) * .005)
            newState.theta =
              Rotation2d(newState.theta.radians - MathUtil.applyDeadband(mechanismController.rightY, .3) * .005)
            robot.arm.moveToState(newState)
          }
        )
      )
    )

    JoystickButton(mechanismController, XboxController.Button.kA.value).onTrue(
      SequentialCommandGroup(
        ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.DOUBLE) }
          .withInterruptBehavior(kCancelIncoming),
        ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.DOUBLE) }
        .andThen(InstantCommand(robot.endEffector::intake))
        .andThen(InstantCommand(robot.endEffector::pistonOn))
      )
    ).onFalse(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.BACK) }
        .withInterruptBehavior(kCancelIncoming)
    )

//    JoystickButton(driveController, XboxController.Button.kBack.value).onTrue(
//      AutoBalance.create(robot.drive)
//    )

    JoystickButton(driveController, XboxController.Button.kStart.value).onTrue(
      InstantCommand({ robot.drive.heading = Rotation2d(0.0) })
    )

//    JoystickButton(driveController, XboxController.Button.kB.value).onTrue(
//      InstantCommand({ DoubleAlign().rightDoubleAlign(robot, driveController).schedule() })
//    )
//
//    JoystickButton(driveController, XboxController.Button.kX.value).onTrue(
//      InstantCommand({ DoubleAlign().leftDoubleAlign(robot, driveController).schedule() })
//    )
  }

  override fun initSendable(builder: SendableBuilder) {
    // TODO if you want i guess
  }
}
