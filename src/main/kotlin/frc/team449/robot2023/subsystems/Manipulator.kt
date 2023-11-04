package frc.team449.robot2023.subsystems

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.IntakeConstants
import frc.team449.robot2023.constants.subsystem.ManipulatorConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class Manipulator(
  private val motor: WrappedMotor
): SubsystemBase() {
  fun intake(): Command {
    return InstantCommand({
      motor.setVoltage(ManipulatorConstants.INTAKE_VOLTAGE)
    })
  }

  fun outtake(): Command {
    return InstantCommand({
      motor.setVoltage(-ManipulatorConstants.INTAKE_VOLTAGE)
    })
  }

  fun stop(): Command {
    return InstantCommand({
      motor.stopMotor()
    })
  }

  override fun periodic() {}

  companion object{
    fun createIntake(): Manipulator {
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