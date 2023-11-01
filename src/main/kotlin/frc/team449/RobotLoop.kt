package frc.team449

import com.pathplanner.lib.server.PathPlannerServer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.routines.RoutineChooser
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.vision.VisionConstants
import frc.team449.robot2023.subsystems.ControllerBindings
import io.github.oblarg.oblog.Logger

/** The main class of the robot, constructs all the subsystems and initializes default commands. */
class RobotLoop : TimedRobot() {

  private val robot = Robot()
  private val routineChooser: RoutineChooser = RoutineChooser(robot)

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

    PathPlannerServer.startServer(5811)


    Logger.configureLoggingAndConfig(robot, false)
    SmartDashboard.putData("Field", robot.field)
    SmartDashboard.putData("Routine Chooser", routineChooser)
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance())

    controllerBinder.bindButtons()

//    robot.light.defaultCommand = BlairAnimation(robot.light)
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()

//    Logger.updateEntries()

    robot.field.robotPose = robot.drive.pose

    robot.field.getObject("bumpers").pose = robot.drive.pose
  }

  override fun autonomousInit() {
    /** At the start of auto we poll the alliance color given by the FMS */
    RobotConstants.ALLIANCE_COLOR = DriverStation.getAlliance()

    /** Every time auto starts, we update the chosen auto command */
    this.autoCommand = routineMap[routineChooser.selected]
    CommandScheduler.getInstance().schedule(this.autoCommand)
  }

  override fun autonomousPeriodic() {}

  override fun teleopInit() {
    VisionConstants.ESTIMATORS.clear()

    if (autoCommand != null) {
      CommandScheduler.getInstance().cancel(autoCommand)
    }

    robot.drive.defaultCommand = robot.driveCommand
  }

  override fun teleopPeriodic() {
  }

  override fun disabledInit() {
    robot.drive.stop()

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
    routineChooser.updateOptions(RobotConstants.ALLIANCE_COLOR == DriverStation.Alliance.Red)
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
