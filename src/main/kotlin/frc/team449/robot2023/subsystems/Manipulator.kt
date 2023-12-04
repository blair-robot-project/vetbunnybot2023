package frc.team449.robot2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.ManipulatorConstants
import frc.team449.system.SparkUtil

class Manipulator(
  motorID: Int
) : SubsystemBase() {

  private val motor = CANSparkMax(motorID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val encoder = motor.encoder

  private var lastVoltage = 0.0

  init {
    SparkUtil.applySparkSettings(
      motor,
      inverted = ManipulatorConstants.INVERTED,
      encoder = encoder,
      unitPerRotation = 1.0,
      gearing = 1.0
    )
  }

  fun intake(): Command {
    return this.runOnce {
      motor.setVoltage(ManipulatorConstants.INTAKE_VOLTAGE)
      lastVoltage = ManipulatorConstants.INTAKE_VOLTAGE
    }
  }

  fun outtake(): Command {
    return this.runOnce {
      motor.setVoltage(-ManipulatorConstants.INTAKE_VOLTAGE)
      lastVoltage = -ManipulatorConstants.INTAKE_VOLTAGE
    }
  }

  fun stop(): Command {
    return this.runOnce {
      motor.stopMotor()
      lastVoltage = 0.0
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Last Voltage", { lastVoltage }, null)
  }

  companion object {
    fun createManipulator(): Manipulator {
      return Manipulator(
        ManipulatorConstants.MOTOR_ID
      )
    }
  }
}
