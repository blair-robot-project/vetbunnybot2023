package frc.team449.control

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N5
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.estimation.OpenCVHelp
import org.photonvision.estimation.PNPResults
import org.photonvision.estimation.VisionEstimation
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
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
) : PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, PhotonCamera(camName), robotToCam) {

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
      multiTagPNPStrategy(cameraResult)
    }
  }

  private fun multiTagPNPStrategy(result: PhotonPipelineResult): Optional<EstimatedRobotPose> {
    // Arrays we need declared up front
    val visCorners = ArrayList<TargetCorner>()
    val knownVisTags = ArrayList<AprilTag>()
    val fieldToCams = ArrayList<Pose3d>()
    val fieldToCamsAlt = ArrayList<Pose3d>()
    val usedTargets = ArrayList<PhotonTrackedTarget>()
    if (result.getTargets().size < 2) {
      // Run fallback strategy instead
      return lowestAmbiguityStrategy(result)
    }
    for (target: PhotonTrackedTarget in result.getTargets()) {
      val tagPoseOpt = tagLayout.getTagPose(target.fiducialId)
      if (tagPoseOpt.isEmpty) {
        reportFiducialPoseError(target.fiducialId)
        continue
      }
      val tagPose = tagPoseOpt.get()

      usedTargets.add(target)

      visCorners.addAll(target.detectedCorners)
      // actual layout poses of visible tags -- not exposed, so have to recreate
      knownVisTags.add(AprilTag(target.fiducialId, tagPose))
      fieldToCams.add(tagPose.transformBy(target.bestCameraToTarget.inverse()))
      fieldToCamsAlt.add(tagPose.transformBy(target.alternateCameraToTarget.inverse()))
    }
    val cameraMatrixOpt = camera.cameraMatrix
    val distCoeffsOpt = camera.distCoeffs
    val hasCalibData = cameraMatrixOpt.isPresent && distCoeffsOpt.isPresent

    // multi-target solvePNP
    return if (hasCalibData && visCorners.size == knownVisTags.size * 4 && knownVisTags.isNotEmpty()) {
      val cameraMatrix = cameraMatrixOpt.get()
      val distCoeffs = distCoeffsOpt.get()
      val pnpResults = estimateCamPosePNP(cameraMatrix, distCoeffs, visCorners, knownVisTags)
      val best = Pose3d()
        .plus(pnpResults.best) // field-to-camera
        .plus(robotToCam.inverse()) // field-to-robot
      Optional.of(
        EstimatedRobotPose(best, result.timestampSeconds, usedTargets)
      )
    } else {
      Optional.empty()
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
        mutableListOf(lowestAmbiguityTarget)
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

  private fun estimateCamPosePNP(
    cameraMatrix: Matrix<N3, N3>,
    distCoeffs: Matrix<N5, N1>,
    corners: List<TargetCorner>,
    knownTags: List<AprilTag>
  ): PNPResults {
    // single-tag pnp
    return if (corners.size == 4) {
      val camToTag = OpenCVHelp.solvePNP_SQUARE(
        cameraMatrix,
        distCoeffs,
        VisionEstimation.kTagModel.getFieldVertices(knownTags[0].pose),
        corners
      )
      val bestTagToCam = Transform3d(
        camToTag.best.translation,
        Rotation3d(
          camToTag.best.x,
          camToTag.best.y,
          driveHeading!!.radians + robotToCam.rotation.z - knownTags[0].pose.rotation.z
        )
      ).inverse()
      val altTagToCam = Transform3d(
        camToTag.alt.translation,
        Rotation3d(
          camToTag.alt.x,
          camToTag.alt.y,
          driveHeading!!.radians + robotToCam.rotation.z - knownTags[0].pose.rotation.z
        )
      ).inverse()
      val bestPose: Pose3d = knownTags[0].pose.transformBy(bestTagToCam)
      var altPose = Pose3d()
      if (camToTag.ambiguity != 0.0) altPose = knownTags[0].pose.transformBy(altTagToCam)
      SmartDashboard.putNumberArray(
        "multiTagBest_internal",
        doubleArrayOf(
          bestTagToCam.x,
          bestTagToCam.y,
          bestTagToCam.z,
          bestTagToCam.rotation.quaternion.w,
          bestTagToCam.rotation.quaternion.x,
          bestTagToCam.rotation.quaternion.y,
          bestTagToCam.rotation.quaternion.z
        )
      )
      val o = Pose3d()
      PNPResults(
        Transform3d(o, bestPose),
        Transform3d(o, altPose),
        camToTag.ambiguity,
        camToTag.bestReprojErr,
        camToTag.altReprojErr
      )
    } else {
      val objectTrls = java.util.ArrayList<Translation3d>()
      for (tag in knownTags) objectTrls.addAll(VisionEstimation.kTagModel.getFieldVertices(tag.pose))
      val camToOrigin = OpenCVHelp.solvePNP_SQPNP(cameraMatrix, distCoeffs, objectTrls, corners)
      PNPResults(
        camToOrigin.best.inverse(),
        camToOrigin.alt.inverse(),
        camToOrigin.ambiguity,
        camToOrigin.bestReprojErr,
        camToOrigin.altReprojErr
      )
    }
  }
}
