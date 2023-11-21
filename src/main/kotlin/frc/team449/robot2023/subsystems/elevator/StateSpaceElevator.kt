package frc.team449.robot2023.subsystems.elevator

import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.subsystem.ElevatorConstants
import frc.team449.system.motor.WrappedMotor
import kotlin.math.abs

open class StateSpaceElevator(
  private val motor: WrappedMotor,
  private val loop: LinearSystemLoop<N2, N1, N1>,
  private val constraints: TrapezoidProfile.Constraints
  ): SubsystemBase() {

  protected val mech: Mechanism2d = Mechanism2d(1.5, 2.0)
  protected val elevatorVisual: MechanismLigament2d

  /** Position first, velocity second (Units: meters or meters/sec) */
  var lastDesiredGoal = TrapezoidProfile.State(0.0, 0.0)
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

    loop.reset(VecBuilder.fill(currentState.first, currentState.second))

    this.defaultCommand = this.run {
      loop.setNextR(desiredState.first, 0.0)
      loop.correct(VecBuilder.fill(currentState.first))
      loop.predict(RobotConstants.LOOP_TIME)

      motor.setVoltage(loop.getU(0) + ElevatorConstants.kSG)
    }
  }

  private fun isFinished(goal: Double, epsilon: Double = 1e-6): Boolean {
    return if (abs(desiredState.first - goal) < epsilon) true
    else false
  }

  private fun moveToPos(distance: Double): Command {
    return this.run {
      lastDesiredGoal = TrapezoidProfile(
        constraints,
        TrapezoidProfile.State(distance, 0.0),
        lastDesiredGoal
      ).calculate(RobotConstants.LOOP_TIME)

      desiredState = Pair(lastDesiredGoal.position, lastDesiredGoal.velocity)

      loop.setNextR(desiredState.first, desiredState.second)
      loop.correct(VecBuilder.fill(currentState.first))
      loop.predict(RobotConstants.LOOP_TIME)

      val voltage = loop.getU(0) + ElevatorConstants.kSG

      motor.setVoltage(voltage)
      println(voltage)
    }.until{ isFinished(distance) }
      .andThen(
      InstantCommand({ desiredState = Pair(distance, 0.0) })
    )
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
    return this.run { motor.setVoltage(ElevatorConstants.kSG) }
  }

  fun stop(): Command{
    return this.runOnce {
      motor.stopMotor()
    }
  }

  override fun periodic() {
    currentState = Pair(loop.getXHat(0), loop.getXHat(1))

    elevatorVisual.length = ElevatorConstants.MIN_LENGTH + currentState.first
    SmartDashboard.putData("Elevator Visual", mech)
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.addDoubleProperty("Last motor voltage", { motor.lastVoltage }, {})
    builder.addDoubleProperty("Current Motor Pos", { currentState.first }, {})
    builder.addDoubleProperty("Current Motor Vel", { currentState.second }, {})
    builder.addDoubleProperty("Desired Motor Pos", { desiredState.first }, {})
    builder.addDoubleProperty("Desired Motor Vel", { desiredState.second }, {})
    builder.addDoubleProperty("kS + kG", { ElevatorConstants.kSG }, { value -> ElevatorConstants.kSG = value })
  }

  companion object {
    fun createStateSpaceElevator(): StateSpaceElevator {
      val plant = LinearSystemId.createElevatorSystem(
        DCMotor.getNEO(ElevatorConstants.NUM_MOTORS),
        ElevatorConstants.CARRIAGE_MASS,
        ElevatorConstants.PULLEY_RADIUS,
        ElevatorConstants.EFFECTIVE_GEARING
      )

      val observer = KalmanFilter(
        Nat.N2(),
        Nat.N1(),
        plant,
        VecBuilder.fill(
          ElevatorConstants.MODEL_POS_STDDEV,
          ElevatorConstants.MODEL_VEL_STDDEV
        ),
        VecBuilder.fill(ElevatorConstants.ENCODER_POS_STDDEV),
        RobotConstants.LOOP_TIME
      )

      val controller = LinearQuadraticRegulator(
        plant,
        VecBuilder.fill(
          ElevatorConstants.LQR_POS_TOL,
          ElevatorConstants.LQR_VEL_TOL
        ),
        VecBuilder.fill(ElevatorConstants.LQR_CONTROL_EFFORT_VOLTS),
        RobotConstants.LOOP_TIME
      )

      val loop = LinearSystemLoop(
        plant,
        controller,
        observer,
        ElevatorConstants.MAX_VOLTAGE,
        RobotConstants.LOOP_TIME
      )

      val constraints = TrapezoidProfile.Constraints(
        ElevatorConstants.MAX_VEL,
        ElevatorConstants.MAX_ACC
      )

      return if (RobotBase.isReal()) {
        StateSpaceElevator(
          ElevatorConstants.motor,
          loop,
          constraints
        )
      } else {
        StateSpaceElevatorSim(
          ElevatorConstants.motor,
          loop,
          constraints
        )
      }
    }
  }
}