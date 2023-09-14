package frc.team449.robot2023.subsystems.nodeTracker

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Timer
import frc.team449.robot2023.subsystems.nodeTracker.NodeSelectorIO.NodeSelectorIOInputs
import io.javalin.Javalin
import io.javalin.http.staticfiles.Location
import java.nio.file.Paths
import kotlin.math.ceil
import kotlin.math.max


class NodeSelectorIOServer: NodeSelectorIO {

  private val table = NetworkTableInstance.getDefault().getTable("nodeselector")
  private val nodePublisher = table.getIntegerTopic("node_robot_to_dashboard").publish()
  private val nodeSubscriber = table.getIntegerTopic("node_dashboard_to_robot").subscribe(-1)
  private val timePublisher = table.getIntegerTopic("match_time").publish()
  private val isAutoPublisher = table.getBooleanTopic("is_auto").publish()

  init {
    // Start server
    val app = Javalin.create { config ->
      config.staticFiles.add(
        Paths.get(
          Filesystem.getDeployDirectory().absolutePath.toString(),
          "placementSelector"
        ).toString(),
        Location.EXTERNAL
      )
    }
    app.start(5800)
  }

  override fun updateInputs(inputs: NodeSelectorIOInputs) {
    timePublisher.set(ceil(max(0.0, Timer.getFPGATimestamp())).toLong())
    isAutoPublisher.set(DriverStation.isAutonomous())
    for (value in nodeSubscriber.readQueueValues()) {
      inputs.selectedNode = value.toInt()
    }
  }

  override fun setSelected(selected: Long) {
    nodePublisher.set(selected)
  }
}