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
): SubsystemBase() {
  fun intake(): Command {
    return this.runOnce {
      motor.setVoltage(ManipulatorConstants.INTAKE_VOLTAGE)
    }
  }

  fun outtake(): Command {
    return this.runOnce {
      motor.setVoltage(-ManipulatorConstants.INTAKE_VOLTAGE)
    }
  }

  fun stop(): Command {
    return this.runOnce {
      motor.stopMotor()
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.addDoubleProperty("Manipulator Motor Voltage", { motor.lastVoltage }, {})
  }

  companion object{
    fun createManipulator(): Manipulator {
      return Manipulator(
        createSparkMax(
          "Manipulator Motor",
          ManipulatorConstants.MOTOR_ID,
          NEOEncoder.creator(
            ManipulatorConstants.UPR,
            ManipulatorConstants.GEARING
          ),
          enableBrakeMode = true,
          inverted = ManipulatorConstants.INVERTED
        )
      )
    }
  }
}