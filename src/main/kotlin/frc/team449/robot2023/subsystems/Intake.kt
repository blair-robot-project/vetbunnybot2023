package frc.team449.robot2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.robot2023.constants.subsystem.IntakeConstants
import frc.team449.system.SparkUtil

class Intake(
  val piston: DoubleSolenoid,
  motorID: Int
) : SubsystemBase() {

  private val motor = CANSparkMax(motorID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val encoder = motor.encoder

  private var lastVoltage = 0.0

  init {
    SparkUtil.applySparkSettings(
      motor,
      inverted = IntakeConstants.INVERTED,
      encoder = encoder,
      unitPerRotation = 1.0,
      gearing = 1.0
    )
  }

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
      lastVoltage = IntakeConstants.INTAKE_VOLTAGE
    }
  }

  fun outtake(): Command {
    return this.runOnce {
      motor.setVoltage(-IntakeConstants.INTAKE_VOLTAGE)
      lastVoltage = -IntakeConstants.INTAKE_VOLTAGE
    }
  }

  fun stop(): Command {
    return this.runOnce {
      motor.stopMotor()
      lastVoltage = 0.0
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Piston")
    builder.addStringProperty("1.1 Piston Status", { piston.get().toString() }, {})

    builder.publishConstString("2.0", "Motor Voltages")
    builder.addDoubleProperty("2.1 Last Voltage", { lastVoltage }, null)

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
      return Intake(
        DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.FORWARD, IntakeConstants.REVERSE),
        IntakeConstants.MOTOR_ID
      )
    }
  }
}
