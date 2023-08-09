package frc.team449.control.obstacleAvoidance

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.auto.HolonomicFollower
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.constants.RobotConstants
import kotlin.math.PI
import kotlin.math.floor
import kotlin.math.hypot

class PPAStar(
  private val drive: SwerveDrive,
  private val constraints: PathConstraints,
  private val finalPosition: Node,
  private val obstacles: List<Obstacle>,
  private val aStarMap: VisGraph,
  private val field: Field2d
) : CommandBase() {
  private var pathDrivingCommand: Command? = null
  private var startPoint: Node? = null

  init {
    aStarMap.addNode(finalPosition)
    for (i in 0 until aStarMap.nodeSize) {
      val endNode = aStarMap.getNode(i)
      aStarMap.addEdge(Edge(finalPosition, endNode), obstacles)
    }
    addRequirements(drive)
  }

  // ----------------------------------------------------------------------------
  // Pre-schedule setup code.
  override fun initialize() {
    /*
     * Add starting position to map
     */

    // Create the starting position
    startPoint = if (RobotConstants.ALLIANCE_COLOR == Alliance.Blue) {
      Node(drive.pose)
    } else {
      val blueStart = Pose2d(
        FieldConstants.FIELD_LENGTH_METERS - drive.pose.x,
        drive.pose.y,
        drive.heading.plus(Rotation2d(PI))
      )
      Node(blueStart)
    }

    // What will be our trajectory for path planner to follow
    var trajectory: PathPlannerTrajectory?

    // Adds start point (current position on the field)
    startPoint?.let { aStarMap.addNode(it) }

    /*
     * START OF FINDING PATH
     */

    // Connects our starting point to all other nodes on the field (this does check
    // for obstacles)
    for (i in 0 until aStarMap.nodeSize) {
      val endNode = aStarMap.getNode(i)
      aStarMap.addEdge(Edge(startPoint!!, endNode), obstacles)
    }
    // Finds the path using A*
    val fullPath: List<Node?> = aStarMap.findPath(startPoint, finalPosition) ?: return

    // Returns if no path is found

    // Gets speed of robot
    val chassisSpeeds = drive.currentSpeeds
    val fieldSpeeds = Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
      .rotateBy(drive.heading)
    val robotSpeeds = ChassisSpeeds(fieldSpeeds.x, fieldSpeeds.y, chassisSpeeds.omegaRadiansPerSecond)
    val startingSpeed = hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)

    // Gets heading that the robot needs to go to reach first point
    var heading = Rotation2d(
      fullPath[1]!!.x -
        startPoint!!.x,
      fullPath[1]!!.y - startPoint!!.y
    )
    val fullPathPoints = ArrayList<PathPoint>()

    /*
     * Takes our path found above and sends it to an ArrayList of Path Planner
     * Points. Also adds in midpoints
     * I would recommend commenting out addMidPoints at first when first testing
     * code
     */

    // Find path between points
    for (i in fullPath.indices) {
      if (i == 0) {
        fullPathPoints.add(
          PathPoint(
            Translation2d(startPoint!!.x, startPoint!!.y),
            heading,
            finalPosition.holRot,
            startingSpeed
          )
        )
        addMidPoints(fullPathPoints, fullPath, i)
      } else if (i + 1 == fullPath.size) {
        heading = Rotation2d(
          fullPath[i]!!.x - fullPath[i - 1]!!.x,
          fullPath[i]!!.y - fullPath[i - 1]!!.y
        )
        fullPathPoints.add(
          PathPoint(
            Translation2d(finalPosition.x, finalPosition.y),
            heading,
            finalPosition.holRot
          )
        )
      } else {
        heading = Rotation2d(
          fullPath[i + 1]!!.x - fullPath[i]!!.x,
          fullPath[i + 1]!!.y - fullPath[i]!!.y
        )
        fullPathPoints.add(
          PathPoint(
            Translation2d(fullPath[i]!!.x, fullPath[i]!!.y),
            heading,
            finalPosition.holRot
          )
        )
        addMidPoints(fullPathPoints, fullPath, i)
      }
    }

    // Declare an array to hold PathPoint objects made from all other points
    // specified in constructor.
    fullPathPoints.forEach {
      println(it.position)
      println(it.holonomicRotation)
    }
    trajectory = PathPlanner.generatePath(constraints, fullPathPoints)
    // Display Trajectory
    // Change trajectory based on alliance color
    trajectory = AutoUtil.transformForAlliancePP(
      mutableListOf(trajectory)
    ) { RobotConstants.ALLIANCE_COLOR == Alliance.Red }[0]
    pathDrivingCommand = HolonomicFollower(
      drive,
      trajectory,
      poseTol = Pose2d(0.02, 0.02, Rotation2d.fromDegrees(2.0))
    )
    pathDrivingCommand!!.schedule()

    field.getObject("PPAStar Path").setTrajectory(trajectory)
  }

  override fun isFinished(): Boolean {
    return pathDrivingCommand == null || !pathDrivingCommand!!.isScheduled
  }

  override fun end(interrupted: Boolean) {
    if (interrupted) {
      pathDrivingCommand!!.cancel()
    }
    drive.stop()
  }

  // Adds MidPoints so that Bézier curve doesn't curve into obstacle
  private fun addMidPoints(fullPathPoints: ArrayList<PathPoint>, fullPath: List<Node?>, i: Int) {
    val distance = hypot(
      fullPath[i + 1]!!.x - fullPath[i]!!.x,
      fullPath[i + 1]!!.y - fullPath[i]!!.y
    )

    // Adjust distance / x to have more or less midpoints lower the x the more midpoints
    val midpoints = floor(distance / 1.45).toInt()
    val heading = Rotation2d(
      fullPath[i + 1]!!.x - fullPath[i]!!.x,
      fullPath[i + 1]!!.y - fullPath[i]!!.y
    )
    for (j in 0 until midpoints) {
      fullPathPoints.add(
        PathPoint(
          Translation2d(
            fullPath[i]!!.x +
              (fullPath[i + 1]!!.x - fullPath[i]!!.x) * ((j + 1.0) / (midpoints + 1.0)),
            (
              fullPath[i]!!.y +
                (fullPath[i + 1]!!.y - fullPath[i]!!.y) * ((j + 1.0) / (midpoints + 1.0))
              )
          ),
          heading,
          finalPosition.holRot
        )
      )
    }
  }
}
