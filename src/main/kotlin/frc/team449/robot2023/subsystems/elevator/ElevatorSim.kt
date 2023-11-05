package frc.team449.robot2023.subsystems.elevator

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.subsystem.ElevatorConstants
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.Loggable

class ElevatorSim(
  private val motor: WrappedMotor,
  controller: ProfiledPIDController,
  feedforward: ElevatorFeedforward
): Elevator(motor, controller, feedforward), Loggable {

  private val elevatorSim = TiltedElevatorSim(
    DCMotor.getNEO(ElevatorConstants.NUM_MOTORS),
    ElevatorConstants.GEARING,
    ElevatorConstants.CARRIAGE_MASS,
    ElevatorConstants.PULLEY_RADIUS,
    0.0,
    ElevatorConstants.HIGH_DISTANCE,
    true,
    angle = ElevatorConstants.ANGLE
  )

  var currentDraw = 0.0

  override fun periodic() {
    elevatorSim.setInputVoltage(motor.lastVoltage)
    elevatorSim.update(RobotConstants.LOOP_TIME)

    currentState = Pair(elevatorSim.positionMeters, elevatorSim.velocityMetersPerSecond)
    currentDraw = elevatorSim.currentDrawAmps

    elevatorVisual.length = ElevatorConstants.MIN_LENGTH + currentState.first
    SmartDashboard.putData("Elevator Visual", mech)
  }

  override fun initSendable(builder: SendableBuilder) {
    super.initSendable(builder)
    builder.addDoubleProperty("Simulated current Draw", { currentDraw }, {})
  }

}