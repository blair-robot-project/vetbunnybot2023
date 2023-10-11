package frc.team449.robot2023.subsystems.endEffector

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.EndEffectorConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log

class EndEffector(
  private val intakeMotor: WrappedMotor,
  val chooserPiston: DoubleSolenoid,
  private val sensor: DigitalInput
) : SubsystemBase() {

  @Log(name = "Cone/Cube in End Effector")
  private var sensorVal = false

  // forward is cone, reverse is cube
  fun pistonOn() {
    chooserPiston.set(DoubleSolenoid.Value.kForward)
  }

  fun pistonRev() {
    chooserPiston.set(DoubleSolenoid.Value.kReverse)
  }

  fun intake() {
    intakeMotor.setVoltage(EndEffectorConstants.INTAKE_VOLTAGE)
  }

  fun holdIntake() {
    intakeMotor.setVoltage(EndEffectorConstants.HOLD_VOLTAGE)
  }

  fun throwCube() {
    intakeMotor.setVoltage(-12.0)
  }

  fun strongHoldIntake() {
    intakeMotor.setVoltage(9.0)
  }

  fun intakeReverse() {
    intakeMotor.setVoltage(EndEffectorConstants.REVERSE_INTAKE_VOLTAGE)
  }

  fun stop() {
    intakeMotor.set(0.0)
  }

  fun autoReverse() {
    intakeMotor.setVoltage(-1.25)
  }

  override fun periodic() {
    sensorVal = sensor.get()
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.addBooleanProperty("sensor", { sensor.get() }, null)
    builder.addDoubleProperty("piston", { if (chooserPiston.get() == DoubleSolenoid.Value.kForward) 1.0 else 0.0 }, null)
    builder.addDoubleProperty("motor", { intakeMotor.lastVoltage }, null)
  }

  companion object {
    fun createEndEffector(): EndEffector {
      val piston = DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        EndEffectorConstants.FORWARD_CHANNEL,
        EndEffectorConstants.REVERSE_CHANNEL
      )

      val motor = createSparkMax(
        "End Effector Motor",
        EndEffectorConstants.MOTOR_ID,
        NEOEncoder.creator(
          EndEffectorConstants.MOTOR_UPR,
          EndEffectorConstants.MOTOR_GEARING
        ),
        inverted = false,
        currentLimit = EndEffectorConstants.MOTOR_CURR_LIM
      )

      val sensor = DigitalInput(
        EndEffectorConstants.SENSOR_CHANNEL
      )

      return EndEffector(
        motor,
        piston,
        sensor
      )
    }
  }
}
