package frc.team449

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.routines.RoutineChooser
import frc.team449.robot2023.commands.light.BlairChasing
import frc.team449.robot2023.commands.light.BreatheHue
import frc.team449.robot2023.commands.light.Rainbow
import frc.team449.robot2023.constants.vision.VisionConstants
import frc.team449.robot2023.subsystems.ControllerBindings
import monologue.Logged
import monologue.Monologue
import monologue.Monologue.LogBoth
import kotlin.jvm.optionals.getOrNull

/** The main class of the robot, constructs all the subsystems and initializes default commands. */
class RobotLoop : TimedRobot(), Logged {

  @LogBoth
  private val robot = Robot()

  private val routineChooser: RoutineChooser = RoutineChooser(robot)

  @LogBoth
  private val field = robot.field

  private var autoCommand: Command? = null
  private var routineMap = hashMapOf<String, Command>()
  private val controllerBinder = ControllerBindings(robot.driveController, robot.mechController, robot)

  override fun robotInit() {
    // Yes this should be a print statement, it's useful to know that robotInit started.
    println("Started robotInit.")

    if (RobotBase.isSimulation()) {
      // Don't complain about joysticks if there aren't going to be any
      DriverStation.silenceJoystickConnectionWarning(true)
//      val instance = NetworkTableInstance.getDefault()
//      instance.stopServer()
//      instance.startClient4("localhost")
    }

    println("Generating Auto Routines : ${Timer.getFPGATimestamp()}")
    routineMap = routineChooser.routineMap()
    println("DONE Generating Auto Routines : ${Timer.getFPGATimestamp()}")

    SmartDashboard.putData("Routine Chooser", routineChooser)
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance())
    SmartDashboard.putBoolean("Enable Logging?", false)

    robot.light.defaultCommand = BlairChasing(robot.light)

    controllerBinder.bindButtons()

    Monologue.setupLogging(this, "/Monologuing")
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()

//    Logger.updateEntries()

    robot.field.robotPose = robot.drive.pose

    robot.field.getObject("bumpers").pose = robot.drive.pose

    if (SmartDashboard.getBoolean("Enable Logging?", false)) {
      Monologue.update()
    } else {
      Monologue.updateFileLog()
    }
  }

  override fun autonomousInit() {
    /** Every time auto starts, we update the chosen auto command */
    this.autoCommand = routineMap[routineChooser.selected]
    CommandScheduler.getInstance().schedule(this.autoCommand)

    if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
      BreatheHue(robot.light, 0).schedule()
    } else {
      BreatheHue(robot.light, 95).schedule()
    }
  }

  override fun autonomousPeriodic() {}

  override fun teleopInit() {
    VisionConstants.ESTIMATORS.clear()

    if (autoCommand != null) {
      CommandScheduler.getInstance().cancel(autoCommand)
    }

    (robot.light.currentCommand?: InstantCommand()).cancel()

    robot.drive.defaultCommand = robot.driveCommand
  }

  override fun teleopPeriodic() {
  }

  override fun disabledInit() {
    robot.drive.stop()

    Rainbow(robot.light).schedule()

//    if (VisionConstants.ESTIMATORS.isEmpty()) {
//      VisionConstants.ESTIMATORS.add(
//        VisionEstimator(
//          VisionConstants.TAG_LAYOUT,
//          "Spinel",
//          VisionConstants.robotToCamera
//        )
//      )
//    } else {
//      VisionConstants.ESTIMATORS[0] =
//        VisionEstimator(
//          VisionConstants.TAG_LAYOUT,
//          "Spinel",
//          VisionConstants.robotToCamera
//        )
//    }
  }

  override fun disabledPeriodic() {
    routineChooser.updateOptions(DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red)
  }

  override fun testInit() {
    if (autoCommand != null) {
      CommandScheduler.getInstance().cancel(autoCommand)
    }
  }

  override fun testPeriodic() {}

  override fun simulationInit() {}

  override fun simulationPeriodic() {}
}
