package frc.team449.robot2023

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.control.holonomic.SwerveOrthogonalCommand
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.subsystems.Intake.Companion.createIntake
import frc.team449.robot2023.subsystems.Manipulator.Companion.createManipulator
import frc.team449.robot2023.subsystems.elevator.Elevator.Companion.createStateSpaceElevator
import frc.team449.system.AHRS
import monologue.Logged
import monologue.Monologue.LogBoth

class Robot : RobotBase(), Logged {

  val driveController = XboxController(0)

  val mechController = XboxController(1)

  val ahrs = AHRS(SPI.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  @LogBoth
  override val powerDistribution: PowerDistribution = PowerDistribution(
    RobotConstants
      .PDH_CAN,
    PowerDistribution.ModuleType.kRev
  )

  @LogBoth
  override val drive = SwerveDrive.createSwerve(ahrs, field)

  @LogBoth
  override val driveCommand = SwerveOrthogonalCommand(drive, driveController)

  @LogBoth
  val intake = createIntake()

  @LogBoth
  val manipulator = createManipulator()

  @LogBoth
  val elevator = createStateSpaceElevator(this)

//  val light = Light.createLight()
//
//  val infrared = DigitalInput(RobotConstants.IR_CHANNEL)
}
