package frc.team449.robot2023.subsystems

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.IntakeConstants
import frc.team449.system.encoder.NEOEncoder

import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class Intake(
  private val piston: DoubleSolenoid,
  private val motor: WrappedMotor,
): SubsystemBase() {
  fun extend(): Command{
    return InstantCommand({ piston.set(DoubleSolenoid.Value.kForward) })
  }

  fun retract(): Command{
    return InstantCommand({ piston.set(DoubleSolenoid.Value.kReverse)})
  }

  fun intake(): Command{
    return InstantCommand({
      motor.setVoltage(IntakeConstants.INTAKE_VOLTAGE)
    })
  }

  fun outtake(): Command{
    return InstantCommand({
      motor.setVoltage(-IntakeConstants.INTAKE_VOLTAGE)
    })
  }

  fun stop(): Command{
    return InstantCommand({
      motor.stopMotor()
    })
  }

  override fun periodic() {}

  companion object{
    fun createIntake(): Intake {
      return Intake(
        DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          IntakeConstants.FORWARD,
          IntakeConstants.REVERSE
        ),
        createSparkMax(
          "Intake Motor",
          IntakeConstants.MOTOR_ID,
          NEOEncoder.creator(
            IntakeConstants.UPR,
            IntakeConstants.GEARING
          ),
          enableBrakeMode = true,
          inverted = IntakeConstants.INVERTED
        )
      )
    }
  }
}