package frc.team449.robot2023.subsystems

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import frc.team449.robot2023.constants.subsystem.IntakeConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class Intake(
  val piston: DoubleSolenoid,
  private val motor: WrappedMotor
) : SubsystemBase() {

  fun extend(): Command {
    return ConditionalCommand(
      SequentialCommandGroup(
        this.runOnce {
          piston.set(DoubleSolenoid.Value.kForward)
        },
        WaitCommand(IntakeConstants.EXTENSION_TIME)
      ),
      InstantCommand()
    ) { piston.get() != DoubleSolenoid.Value.kForward }
  }

  fun retract(): Command {
    return this.runOnce { piston.set(DoubleSolenoid.Value.kReverse) }
  }

  fun intake(): Command {
    return this.runOnce {
      motor.setVoltage(IntakeConstants.INTAKE_VOLTAGE)
    }
  }

  fun outtake(): Command {
    return this.runOnce {
      motor.setVoltage(-IntakeConstants.INTAKE_VOLTAGE)
    }
  }

  fun stop(): Command {
    return this.runOnce {
      motor.stopMotor()
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Piston")
    builder.addStringProperty("1.1 Piston Status", { piston.get().toString() }, {})

    builder.publishConstString("2.0", "Motor Voltages")
    builder.addDoubleProperty("2.1 Last Voltage", { motor.lastVoltage }, null)

    if (RobotBase.isSimulation()) {
      builder.publishConstString("3.0", "Advantage Scope 3D Pos")
      builder.addDoubleArrayProperty(
        "3.1 Intake 3D Position",
        {
          if (piston.get() == DoubleSolenoid.Value.kForward) {
            doubleArrayOf(
              -0.065,
              0.0,
              0.245,
              0.9411613,
              0.0,
              0.3379578,
              0.0
            )
          } else {
            doubleArrayOf(
              0.0,
              0.0,
              0.0,
              1.0,
              0.0,
              0.0,
              0.0
            )
          }
        },
        null
      )
    }
  }

  companion object {
    fun createIntake(): Intake {
      val motor = createSparkMax(
        "Intake",
        IntakeConstants.MOTOR_ID,
        NEOEncoder.creator(
          1.0,
          1.0
        ),
        inverted = IntakeConstants.INVERTED,
        currentLimit = IntakeConstants.CURRENT_LIM
      )

      return Intake(
        DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.FORWARD, IntakeConstants.REVERSE),
        motor
      )
    }
  }
}
