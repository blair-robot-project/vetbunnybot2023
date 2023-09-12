package frc.team449.robot2023.subsystems.nodeTracker

import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.Filesystem
import io.javalin.Javalin
import io.javalin.http.staticfiles.Location
import java.nio.file.Paths


class NodeSelectorIOServer() {

  private var nodePublisher: IntegerPublisher? = null
  private var nodeSubscriber: IntegerSubscriber? = null
  private var coneTippedPublisher: BooleanPublisher? = null
  private var coneTippedSubscriber: BooleanSubscriber? = null
  private var timePublisher: IntegerPublisher? = null
  private var isAutoPublisher: BooleanPublisher? = null

  fun setUp() {
    println("[Init] Creating NodeSelectorIOServer")

    val table = NetworkTableInstance.getDefault().getTable("nodeselector")
    nodePublisher = table.getIntegerTopic("node_robot_to_dashboard").publish()
    nodeSubscriber = table.getIntegerTopic("node_dashboard_to_robot").subscribe(-1)
    coneTippedPublisher = table.getBooleanTopic("cone_tipped_robot_to_dashboard").publish()
    coneTippedSubscriber = table.getBooleanTopic("cone_tipped_dashboard_to_robot").subscribe(false)
    timePublisher = table.getIntegerTopic("match_time").publish()
    isAutoPublisher = table.getBooleanTopic("is_auto").publish()

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
}