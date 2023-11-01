package frc.team449.robot2023.subsystems

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.XboxController
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
  }

  override fun initSendable(builder: SendableBuilder) {
    // TODO if you want i guess
  }
}
