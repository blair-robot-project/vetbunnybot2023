package frc.team449.robot2023

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.control.holonomic.SwerveOrthogonalCommand
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.subsystems.arm.Arm.Companion.createArm
import frc.team449.robot2023.subsystems.arm.ArmSim.Companion.createArmSim
import frc.team449.robot2023.subsystems.endEffector.EndEffector.Companion.createEndEffector
import frc.team449.robot2023.subsystems.groundIntake.GroundIntake.Companion.createGroundIntake
import frc.team449.system.AHRS
import io.github.oblarg.oblog.annotations.Log

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

  @Log(name = "Double Joint Arm")
  val arm = if (isReal()) createArm() else createArmSim()

  @Log(name = "End Effector")
  val endEffector = createEndEffector()

  @Log(name = "Ground Intake")
  val groundIntake = createGroundIntake()

//  val tracker = createObjTracker(this)

//  val light = Light.createLight()
//
//  val infrared = DigitalInput(RobotConstants.IR_CHANNEL)
}
