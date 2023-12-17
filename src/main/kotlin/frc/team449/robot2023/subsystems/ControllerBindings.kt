package frc.team449.robot2023.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.robot2023.Robot
import frc.team449.robot2023.commands.characterization.Characterization
import frc.team449.robot2023.commands.light.BlairChasing
import frc.team449.robot2023.commands.light.BreatheHue
import frc.team449.robot2023.commands.light.ElevatorMoving
import frc.team449.robot2023.constants.RobotConstants
import kotlin.math.PI
import kotlin.math.abs

class ControllerBindings(
  private val driveController: XboxController,
  private val mechanismController: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {
    // slow drive
    Trigger { driveController.rightTriggerAxis >= .75 }.onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 1.0 }).andThen(
        InstantCommand({ robot.drive.maxRotSpeed = PI / 4 })
      )
    ).onFalse(
      InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED }).andThen(
        InstantCommand({ robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED })
      )
    )

    JoystickButton(mechanismController, XboxController.Button.kX.value).onTrue(
      SequentialCommandGroup(
        robot.intake.extend(),
        robot.elevator.low()
      )
    )

    JoystickButton(driveController, XboxController.Button.kStart.value).onTrue(
      InstantCommand({
        robot.drive.heading = Rotation2d()
      })
    )

    JoystickButton(mechanismController, XboxController.Button.kY.value).onTrue(
      SequentialCommandGroup(
        robot.intake.extend(),
        robot.elevator.high(),
      )
    )

    JoystickButton(mechanismController, XboxController.Button.kB.value).onTrue(
      SequentialCommandGroup(
        robot.intake.extend(),
        robot.elevator.stow()
      )
    )

    JoystickButton(mechanismController, XboxController.Button.kA.value).onTrue(
      SequentialCommandGroup(
        robot.elevator.summaryStats(),
        robot.elevator.tuneKG()
      )
    )

    JoystickButton(mechanismController, XboxController.Button.kBack.value).onTrue(
      robot.elevator.tuneKS()
    )

    JoystickButton(mechanismController, XboxController.Button.kRightBumper.value).onTrue(
      robot.intake.extend()

    )

    JoystickButton(mechanismController, XboxController.Button.kLeftBumper.value).onTrue(
      robot.intake.retract()
    )

    JoystickButton(driveController, XboxController.Button.kA.value).onTrue(
      Characterization(
        robot.drive,
        true,
        "swerve drive",
        robot.drive::setVoltage,
        robot.drive::getModuleVel
      ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    ).onFalse(
      robot.driveCommand
    )

    Trigger { mechanismController.rightTriggerAxis > 0.8 }.onTrue(
      ParallelCommandGroup(
        robot.intake.intake(),
        robot.manipulator.intake(),
        BreatheHue(robot.light, 30)
      )
    ).onFalse(
      ParallelCommandGroup(
        robot.intake.stop(),
        robot.manipulator.hold(),
        BlairChasing(robot.light)
      )
    )

    Trigger { abs(mechanismController.leftY) > 0.175 }.onTrue(
      robot.elevator.manualMovement({ -mechanismController.leftY })
    ).onFalse(
      robot.elevator.defaultCommand
    )

    Trigger { mechanismController.leftTriggerAxis > 0.8 }.onTrue(
      ParallelCommandGroup(
        robot.intake.outtake(),
        robot.manipulator.outtake(),
        BreatheHue(robot.light, 160)
      )
    ).onFalse(
      ParallelCommandGroup(
        robot.intake.stop(),
        robot.manipulator.stop(),
        BlairChasing(robot.light)
      )
    )

    Trigger { robot.intake.piston.get() == DoubleSolenoid.Value.kForward }.onTrue(
      BreatheHue(robot.light, 60)
    ).onFalse(
      BlairChasing(robot.light)
    )

    Trigger { robot.elevator.inMotion }.onTrue(
      ElevatorMoving(robot)
    ).onFalse(
      SequentialCommandGroup(
        WaitCommand(0.5),
        BlairChasing(robot.light)
      )
    )

  }
}
