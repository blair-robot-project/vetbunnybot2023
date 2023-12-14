package frc.team449.robot2023.constants.vision

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Filesystem
import frc.team449.control.VisionEstimator
import org.photonvision.estimation.TargetModel

/** Constants that have anything to do with vision */
object
VisionConstants {
  /** How the tags are laid out on the field (their locations and ids) */
  private val TEST_TAG_LAYOUT = AprilTagFieldLayout(
    listOf(
      AprilTag(3, Pose3d())
    ),
    16.4846,
    8.1026
  )

  /** WPILib's AprilTagFieldLayout for the 2023 Charged Up Game */
  val TAG_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout(
    Filesystem.getDeployDirectory().absolutePath.plus("/vision/Bunnybots2023.json")
  )

  /** Robot to Camera distance */
  val robotToCamera = Transform3d(
    Translation3d(Units.inchesToMeters(-11.48657), Units.inchesToMeters(0.0), Units.inchesToMeters(8.3416)),
    Rotation3d(0.0, Units.degreesToRadians(15.0), Units.degreesToRadians(180.0))
  )

  val TAG_MODEL = TargetModel(
    Units.inchesToMeters(6.0),
    Units.inchesToMeters(6.0)
  )

  const val MAX_AMBIGUITY = 0.425

  var MAX_DISTANCE_SINGLE_TAG = 4.0
  var MAX_DISTANCE_MULTI_TAG = 4.5

  /** List of cameras that we want to use */
  val ESTIMATORS: ArrayList<VisionEstimator> = arrayListOf(
//    VisionEstimator(
//      TAG_LAYOUT,
//      "arducam",
//      robotToCamera
//    )
  )

  val ENCODER_TRUST = MatBuilder(Nat.N3(), Nat.N1()).fill(.10, .10, .025)
  val VISION_TRUST = MatBuilder(Nat.N3(), Nat.N1()).fill(.375, .0375, .5)
}
