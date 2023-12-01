package frc.team449.robot2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.IntakeConstants
import frc.team449.system.SparkUtil
import monologue.Logged
import monologue.Monologue
import monologue.Monologue.LogNT

class Intake(
    private val piston: DoubleSolenoid,
    private val motorID: Int
) : SubsystemBase(), Logged {

    private val motor = CANSparkMax(motorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val encoder = motor.encoder

    @LogNT
    private var intakeExtended = false;

    // TODO: check gearing with mechanics
    init {
        Monologue.setupLogging(this, "/Intake")

        SparkUtil.applySparkSettings(
            motor,
            inverted = IntakeConstants.INVERTED,
            encoder = encoder,
            unitPerRotation = IntakeConstants.UPR,
            gearing = IntakeConstants.GEARING,
            offset = Double.NaN
        )
    }

    fun extend(): Command {
        return this.runOnce { piston.set(DoubleSolenoid.Value.kForward); intakeExtended = true }
    }

    fun retract(): Command {
        return this.runOnce { piston.set(DoubleSolenoid.Value.kReverse); intakeExtended = false }
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
