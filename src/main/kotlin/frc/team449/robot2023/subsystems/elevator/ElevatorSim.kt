package frc.team449.robot2023.subsystems.elevator

import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.subsystem.ElevatorConstants
import frc.team449.system.motor.WrappedMotor
import java.util.function.Supplier

class ElevatorSim(
    private val motor: WrappedMotor,
    loop: LinearSystemLoop<N2, N1, N1>
) : Elevator(motor, loop) {

    private val elevatorSim = TiltedElevatorSim(
        DCMotor(
            ElevatorConstants.NOMINAL_VOLTAGE,
            ElevatorConstants.STALL_TORQUE,
            ElevatorConstants.STALL_CURRENT,
            ElevatorConstants.FREE_CURRENT,
            ElevatorConstants.FREE_SPEED,
            ElevatorConstants.NUM_MOTORS
        ),
        ElevatorConstants.EFFECTIVE_GEARING,
        ElevatorConstants.CARRIAGE_MASS,
        ElevatorConstants.PULLEY_RADIUS,
        0.0,
        ElevatorConstants.HIGH_DISTANCE,
        true,
        angle = ElevatorConstants.ANGLE
    )

    override val positionSupplier =
        Supplier { elevatorSim.positionMeters }

    var currentDraw = 0.0

    override fun periodic() {
        elevatorSim.setInputVoltage(motor.lastVoltage)

        elevatorSim.update(RobotConstants.LOOP_TIME)

        currentState = Pair(elevatorSim.positionMeters, elevatorSim.velocityMetersPerSecond)

        currentDraw = elevatorSim.currentDrawAmps

        elevatorVisual.length = ElevatorConstants.MIN_LENGTH + currentState.first
        desiredElevatorVisual.length = ElevatorConstants.MIN_LENGTH + desiredState.first

        SmartDashboard.putData("Elevator Visual", mech)
    }

    override fun initSendable(builder: SendableBuilder) {
        super.initSendable(builder)
        builder.addDoubleProperty("Simulated current Draw", { currentDraw }, {})
    }
}
