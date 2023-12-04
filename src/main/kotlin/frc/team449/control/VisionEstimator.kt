package frc.team449.control

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj.DriverStation
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.Optional
import kotlin.math.abs

/**
 * This class uses normal multi-tag PNP and lowest ambiguity using the gyro rotation
 *  for the internal cam-to-tag transform as a fallback strategy
 */
class VisionEstimator(
  private val tagLayout: AprilTagFieldLayout,
  camName: String,
  private val robotToCam: Transform3d
) : PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhotonCamera(camName), robotToCam) {

  private val camera = PhotonCamera(camName)
  private val reportedErrors: HashSet<Int> = HashSet()
  private var driveHeading: Rotation2d? = null

  fun estimatedPose(currHeading: Rotation2d): Optional<EstimatedRobotPose> {
    driveHeading = currHeading
    return updatePose(camera.latestResult)
  }

  private fun updatePose(cameraResult: PhotonPipelineResult?): Optional<EstimatedRobotPose> {
    // Time in the past -- give up, since the following if expects times > 0
    if (cameraResult!!.timestampSeconds < 0) {
      return Optional.empty()
    }

    // If the pose cache timestamp was set, and the result is from the same timestamp, return an
    // empty result
    if (poseCacheTimestampSeconds > 0 &&
      abs(poseCacheTimestampSeconds - cameraResult.timestampSeconds) < 1e-6
    ) {
      return Optional.empty()
    }

    // Remember the timestamp of the current result used
    poseCacheTimestampSeconds = cameraResult.timestampSeconds

    // If no targets seen, trivial case -- return empty result
    return if (!cameraResult.hasTargets()) {
      Optional.empty()
    } else {
      multiTagOnCoprocStrategy(cameraResult)
    }
  }

  private fun multiTagOnCoprocStrategy(
    result: PhotonPipelineResult
  ): Optional<EstimatedRobotPose> {
    return if (result.multiTagResult.estimatedPose.isPresent) {
      val best_tf = result.multiTagResult.estimatedPose.best
      val best = Pose3d()
        .plus(best_tf) // field-to-camera
        .relativeTo(fieldTags.origin)
        .plus(robotToCam.inverse()) // field-to-robot
      Optional.of(
        EstimatedRobotPose(
          best,
          result.timestampSeconds,
          result.getTargets(),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
        )
      )
    } else {
      lowestAmbiguityStrategy(result)
    }
  }

  /**
   * Return the estimated position of the robot with the lowest position ambiguity from a List of
   * pipeline results.
   *
   * @param result pipeline result
   * @return the estimated position of the robot in the FCS and the estimated timestamp of this
   * estimation.
   */
  private fun lowestAmbiguityStrategy(result: PhotonPipelineResult): Optional<EstimatedRobotPose> {
    var lowestAmbiguityTarget: PhotonTrackedTarget? = null
    var lowestAmbiguityScore = 10.0
    for (target: PhotonTrackedTarget in result.targets) {
      val targetPoseAmbiguity = target.poseAmbiguity
      // Make sure the target is a Fiducial target.
      if (targetPoseAmbiguity != -1.0 && targetPoseAmbiguity < lowestAmbiguityScore) {
        lowestAmbiguityScore = targetPoseAmbiguity
        lowestAmbiguityTarget = target
      }
    }

    // Although there are confirmed to be targets, none of them may be fiducial
    // targets.
    if (lowestAmbiguityTarget == null) return Optional.empty()
    val targetFiducialId = lowestAmbiguityTarget.fiducialId
    val targetPosition = tagLayout.getTagPose(targetFiducialId)
    if (targetPosition.isEmpty) {
      reportFiducialPoseError(targetFiducialId)
      return Optional.empty()
    }

    val cameraToTarget = Transform3d(
      lowestAmbiguityTarget.bestCameraToTarget.translation,
      Rotation3d(
        lowestAmbiguityTarget.bestCameraToTarget.rotation.x,
        lowestAmbiguityTarget.bestCameraToTarget.rotation.y,
        driveHeading!!.radians + robotToCam.rotation.z - targetPosition.get().rotation.z
      )
    )

    return Optional.of(
      EstimatedRobotPose(
        targetPosition
          .get()
          .transformBy(cameraToTarget.inverse())
          .transformBy(robotToCam.inverse()),
        result.timestampSeconds,
        mutableListOf(lowestAmbiguityTarget),
        PoseStrategy.LOWEST_AMBIGUITY
      )
    )
  }

  private fun reportFiducialPoseError(fiducialId: Int) {
    if (!reportedErrors.contains(fiducialId)) {
      DriverStation.reportError(
        "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: $fiducialId",
        false
      )
      reportedErrors.add(fiducialId)
    }
  }
}
