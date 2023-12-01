package frc.team449.robot2023

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.control.holonomic.SwerveOrthogonalCommand
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.subsystems.Intake.Companion.createIntake
import frc.team449.robot2023.subsystems.Manipulator.Companion.createManipulator
import frc.team449.robot2023.subsystems.elevator.Elevator.Companion.createStateSpaceElevator
import frc.team449.system.AHRS
import io.github.oblarg.oblog.annotations.Log
import monologue.Logged
import monologue.Monologue
import monologue.Monologue.LogNT

class Robot : RobotBase() {

    val driveController = XboxController(0)

    val mechController = XboxController(1)

    val ahrs = AHRS(SerialPort.Port.kMXP)

    // Instantiate/declare PDP and other stuff here

    @Log(name = "PDH Logs")
    override val powerDistribution: PowerDistribution = PowerDistribution(
        RobotConstants
            .PDH_CAN,
        PowerDistribution.ModuleType.kRev
    )

    override val drive = SwerveDrive.createSwerve(ahrs, field)

    @Log(name = "Joystick Input")
    override val driveCommand = SwerveOrthogonalCommand(drive, driveController)

    @Log(name = "Elevator")
    val elevator = createStateSpaceElevator()

    val intake = createIntake()

    val manipulator = createManipulator()

//  val light = Light.createLight()
//
//  val infrared = DigitalInput(RobotConstants.IR_CHANNEL)
}
