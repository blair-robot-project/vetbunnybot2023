package frc.team449.robot2023.subsystems.elevator

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.ElevatorConstants
import frc.team449.robot2023.constants.subsystem.ElevatorConstants.motor
import frc.team449.system.motor.WrappedMotor

open class Elevator(
  private val motor: WrappedMotor,
  protected val controller: ProfiledPIDController,
  private var feedforward: ElevatorFeedforward
): SubsystemBase() {

  protected val mech: Mechanism2d = Mechanism2d(1.5, 2.0)
  protected val elevatorVisual: MechanismLigament2d

  /** Position first, velocity second (Units: meters or meters/sec */
  var desiredState = Pair(0.0, 0.0)
  var currentState = Pair(motor.position, motor.velocity)

  init {
    val rootMech = mech.getRoot("elevator", 0.25, 0.25)
    elevatorVisual = rootMech.append(
      MechanismLigament2d(
        "elevator",
        ElevatorConstants.MIN_LENGTH,
        ElevatorConstants.ANGLE,
        ElevatorConstants.WIDTH,
        ElevatorConstants.COLOR
      )
    )

    this.defaultCommand = this.run {
      controller.goal = TrapezoidProfile.State(desiredState.first, 0.0)

      motor.setVoltage(controller.calculate(currentState.first) + ElevatorConstants.kS + ElevatorConstants.kG)
    }
  }

  private fun moveToPos(distance: Double): Command {
    return this.run {
      controller.goal = TrapezoidProfile.State(distance, 0.0)

      desiredState = Pair(controller.setpoint.position, controller.setpoint.velocity)

      if (desiredState.first == controller.goal.position && desiredState.second == controller.goal.velocity)
        motor.setVoltage(controller.calculate(currentState.first) + ElevatorConstants.kS + ElevatorConstants.kG)
      else
        motor.setVoltage(controller.calculate(currentState.first) + feedforward.calculate(desiredState.second))
    }
  }

  fun high(): Command {
    return moveToPos(ElevatorConstants.HIGH_DISTANCE)
  }

  fun low(): Command {
    return moveToPos(ElevatorConstants.LOW_DISTANCE)
  }

  fun stow(): Command{
    return moveToPos(ElevatorConstants.STOW_DISTANCE)
  }

  fun tuneKS(): Command {
    return this.run { motor.setVoltage(ElevatorConstants.kS) }
  }

  fun stop(): Command{
    return this.runOnce {
      motor.stopMotor()
    }
  }

  override fun periodic() {
    currentState = Pair(motor.position, motor.velocity)

    elevatorVisual.length = ElevatorConstants.MIN_LENGTH + currentState.first
    SmartDashboard.putData("Elevator Visual", mech)
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.addDoubleProperty("Current Motor Pos", { currentState.first }, {})
    builder.addDoubleProperty("Current Motor Vel", { currentState.second }, {})
    builder.addDoubleProperty("Desired Motor Pos", { desiredState.first }, {})
    builder.addDoubleProperty("Desired Motor Vel", { desiredState.second }, {})
    builder.addDoubleProperty("Last motor voltage", { motor.lastVoltage }, {})
    builder.addDoubleProperty("kS", { ElevatorConstants.kS }, { value -> ElevatorConstants.kS = value; feedforward = ElevatorFeedforward(value, feedforward.kg, feedforward.kv) })
    builder.addDoubleProperty("kV", { ElevatorConstants.kV }, { value -> ElevatorConstants.kV = value; feedforward = ElevatorFeedforward(feedforward.ks, feedforward.kg, value) })
    builder.addDoubleProperty("kG", { ElevatorConstants.kG }, { value -> ElevatorConstants.kG = value; feedforward = ElevatorFeedforward(feedforward.ks, value, feedforward.kv) })
    builder.addDoubleProperty("kP", { ElevatorConstants.kP }, { value -> ElevatorConstants.kP = value; controller.p = value })
  }

  companion object {
    fun createElevator(): Elevator {
      val controller = ProfiledPIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD,
        TrapezoidProfile.Constraints(ElevatorConstants.MAX_VEL, ElevatorConstants.MAX_ACC)
      )

      val feedforward = ElevatorFeedforward(
        ElevatorConstants.kS,
        ElevatorConstants.kG,
        ElevatorConstants.kV
      )

      return if (RobotBase.isReal()) {
        Elevator(
          motor,
          controller,
          feedforward
        )
      } else {
        ElevatorSim(
          motor,
          controller,
          feedforward
        )
      }
    }
  }
}