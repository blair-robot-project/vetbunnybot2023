package frc.team449.robot2023.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.ElevatorConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class Elevator(
  private val motor: WrappedMotor,
  private val controller: PIDController,
  private val feedforward: SimpleMotorFeedforward,
  private var profile: TrapezoidProfile
): SubsystemBase() {

  private val dt = 0.02
  private var state = TrapezoidProfile.State(0.0, 0.0)

  fun extend(): Command{
    return RepeatCommand(
      InstantCommand({
        profile = TrapezoidProfile(
          TrapezoidProfile.Constraints(ElevatorConstants.MAX_VEL, ElevatorConstants.MAX_ACC),
          TrapezoidProfile.State(ElevatorConstants.EXTEND_DISTANCE, 0.0),
          state
        )
        state = profile.calculate(dt)
        motor.setVoltage(controller.calculate(motor.position, state.position) + feedforward.calculate(state.velocity))
      }).until { controller.atSetpoint() }
    )
  }

  fun retract(): Command{
    return RepeatCommand(
      InstantCommand({
        profile = TrapezoidProfile(
          TrapezoidProfile.Constraints(ElevatorConstants.MAX_VEL, ElevatorConstants.MAX_ACC),
          TrapezoidProfile.State(0.0, 0.0),
          state
        )
        state = profile.calculate(dt)
        motor.setVoltage(controller.calculate(motor.position, state.position) + feedforward.calculate(state.velocity))
      }).until { controller.atSetpoint() }
    )
  }

  override fun periodic() {}

  companion object {
    fun createElevator(): Elevator{
      val motor = createSparkMax(
        "Elevator Motor",
        ElevatorConstants.LEFT_ID,
        NEOEncoder.creator(
          ElevatorConstants.UPR,
          ElevatorConstants.GEARING
        ),
        enableBrakeMode = true,
        inverted = ElevatorConstants.LEFT_INVERTED,
        slaveSparks = mapOf(
          Pair(ElevatorConstants.RIGHT_ID, ElevatorConstants.RIGHT_INVERTED)
        )
      )

      val controller = PIDController(
        ElevatorConstants.KP,
        ElevatorConstants.KI,
        ElevatorConstants.KD
      )
      controller.setTolerance(
        ElevatorConstants.POS_TOL,
        ElevatorConstants.VEL_TOL
      )

      val feedforward = SimpleMotorFeedforward(
        ElevatorConstants.KS,
        ElevatorConstants.KV,
        ElevatorConstants.KA
      )

      val profile = TrapezoidProfile(
        TrapezoidProfile.Constraints(ElevatorConstants.MAX_VEL, ElevatorConstants.MAX_ACC),
        TrapezoidProfile.State(0.0, 0.0),
        TrapezoidProfile.State(0.0, 0.0)
      )

      return Elevator(
        motor,
        controller,
        feedforward,
        profile
      )
    }
  }
}