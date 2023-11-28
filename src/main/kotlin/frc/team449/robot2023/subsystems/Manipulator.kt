package frc.team449.robot2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.ManipulatorConstants
import frc.team449.system.SparkUtil

class Manipulator(
    private val motorID: Int
) : SubsystemBase() {

    private val motor = CANSparkMax(motorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val encoder = motor.encoder

    init {
        SparkUtil.applySparkSettings(
            motor,
            inverted = ManipulatorConstants.INVERTED,
            encoder = encoder,
            unitPerRotation = ManipulatorConstants.UPR,
            gearing = ManipulatorConstants.GEARING,
            offset = Double.NaN
        )
    }

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
        builder.addDoubleProperty("Intake Motor Velocity", { encoder.velocity }, {})
    }

    companion object {
        fun createManipulator(): Manipulator {
            return Manipulator(
                ManipulatorConstants.MOTOR_ID
            )
        }
    }
}
