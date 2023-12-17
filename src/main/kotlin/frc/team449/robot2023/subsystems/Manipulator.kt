package frc.team449.robot2023.subsystems

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.ManipulatorConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class Manipulator(
  private val motor: WrappedMotor
) : SubsystemBase() {

  fun intake(): Command {
    return this.runOnce {
      motor.setVoltage(ManipulatorConstants.INTAKE_VOLTAGE)
    }
  }

  fun hold(): Command {
    return this.runOnce {
      motor.setVoltage(ManipulatorConstants.HOLD_VOLTAGE)
    }
  }

  fun outtake(): Command {
    return this.runOnce {
      motor.setVoltage(ManipulatorConstants.INTAKE_OUT)
    }
  }

  fun stop(): Command {
    return this.runOnce {
      motor.stopMotor()
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Last Voltage", { motor.lastVoltage }, null)
  }

  companion object {
    fun createManipulator(): Manipulator {
      val motor = createSparkMax(
        "Manipulator",
        ManipulatorConstants.MOTOR_ID,
        NEOEncoder.creator(
          1.0,
          1.0
        ),
        inverted = ManipulatorConstants.INVERTED,
        currentLimit = ManipulatorConstants.CURRENT_LIM
      )

      return Manipulator(motor)
    }
  }
}
