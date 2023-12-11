package frc.team449.robot2023.subsystems

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.robot2023.Robot
import kotlin.math.abs

class ControllerBindings(
  private val driveController: XboxController,
  private val mechanismController: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {
    JoystickButton(mechanismController, XboxController.Button.kX.value).onTrue(
      SequentialCommandGroup(
        // How long does it take to extend and retract the intake? Is it the play to have mech manually extend intake right as we enter the
        // community or have it automated such that it extends intake and moves elevator at the same time?
//        robot.intake.extend(),
        robot.elevator.low()
      )
    )

    JoystickButton(mechanismController, XboxController.Button.kY.value).onTrue(
      SequentialCommandGroup(
        robot.intake.extend(),
        robot.elevator.high()
      )
    )

    JoystickButton(mechanismController, XboxController.Button.kB.value).onTrue(
      SequentialCommandGroup(
//        robot.intake.extend(),
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

    Trigger { driveController.rightTriggerAxis > 0.8 }.onTrue(
      ParallelCommandGroup(
        robot.intake.intake(),
        robot.manipulator.intake()
      )
    ).onFalse(
      ParallelCommandGroup(
        robot.intake.stop(),
        robot.manipulator.stop()
      )
    )

    Trigger { abs(mechanismController.leftY) > 0.25 }.onTrue(
      robot.elevator.manualMovement({ -mechanismController.leftY })
    ).onFalse(
      robot.elevator.defaultCommand
    )

    Trigger { driveController.leftTriggerAxis > 0.8 }.onTrue(
      ParallelCommandGroup(
        robot.intake.outtake(),
        robot.manipulator.outtake()
      )
    ).onFalse(
      ParallelCommandGroup(
        robot.intake.stop(),
        robot.manipulator.stop()
      )
    )
  }
}
