package frc.team449.robot2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.IntakeConstants
import frc.team449.system.SparkUtil

class Intake(
  val piston: DoubleSolenoid,
  private val motorID: Int
) : SubsystemBase() {

  private val motor = CANSparkMax(motorID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val encoder = motor.encoder

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
    return this.runOnce { piston.set(DoubleSolenoid.Value.kForward) }
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
    builder.addStringProperty("Intake Piston Status", { piston.get().toString() }, {})

    if (RobotBase.isSimulation()) {
      builder.addDoubleArrayProperty(
        "Intake 3D Position",
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
