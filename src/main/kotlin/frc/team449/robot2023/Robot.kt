package frc.team449.robot2023

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.OrthogonalHolonomicOI.Companion.createOrthogonalHolonomicOI
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.subsystems.arm.Arm.Companion.createArm
import frc.team449.robot2023.subsystems.arm.ArmSim.Companion.createArmSim
import frc.team449.robot2023.subsystems.endEffector.EndEffector.Companion.createEndEffector
import frc.team449.robot2023.subsystems.groundIntake.GroundIntake.Companion.createGroundIntake
import frc.team449.robot2023.subsystems.nodeTracker.ObjectiveTracker.Companion.createObjTracker
import frc.team449.system.AHRS
import io.github.oblarg.oblog.annotations.Log

class Robot : RobotBase() {

  val driveController = XboxController(0)

  val mechanismController = XboxController(1)

  val scoringController = XboxController(2)

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
  override val oi = createOrthogonalHolonomicOI(drive, driveController)

  @Log(name = "Double Joint Arm")
  val arm = if (isReal()) createArm() else createArmSim()

  @Log(name = "End Effector")
  val endEffector = createEndEffector()

  val groundIntake = createGroundIntake()

  val tracker = createObjTracker(this)

//  val light = Light.createLight()
//
//  val infrared = DigitalInput(RobotConstants.IR_CHANNEL)
}
