package frc.team449.robot2023.subsystems

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.robot2023.Robot

class ControllerBindings(
    private val driveController: XboxController,
    private val mechanismController: XboxController,
    private val robot: Robot
) : Sendable {

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

        JoystickButton(mechanismController, XboxController.Button.kX.value).onTrue(
            robot.elevator.low()
        )

        JoystickButton(mechanismController, XboxController.Button.kA.value).onTrue(
            robot.elevator.high()
        )

        JoystickButton(mechanismController, XboxController.Button.kB.value).onTrue(
            robot.elevator.stow()
        )

        JoystickButton(mechanismController, XboxController.Button.kY.value).onTrue(
            SequentialCommandGroup(
                robot.elevator.showSummaryStats(),
                robot.elevator.tuneKG()
            )
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

    override fun initSendable(builder: SendableBuilder) {
        // TODO if you want i guess
    }
}
