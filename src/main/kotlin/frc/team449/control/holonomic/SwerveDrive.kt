package frc.team449.control.holonomic

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.control.VisionEstimator
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.drives.SwerveConstants
import frc.team449.robot2023.constants.vision.VisionConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * A Swerve Drive chassis.
 * @param modules An array of [SwerveModule]s that are on the drivetrain.
 * @param ahrs The gyro that is mounted on the chassis.
 * @param maxLinearSpeed The maximum translation speed of the chassis.
 * @param maxRotSpeed The maximum rotation speed of the chassis.
 * @param cameras The cameras that help estimate the robot's pose.
 * @param field The SmartDashboard [Field2d] widget that shows the robot's pose.
 */
open class SwerveDrive(
  private val modules: List<SwerveModule>,
  private val ahrs: AHRS,
  override var maxLinearSpeed: Double,
  override var maxRotSpeed: Double,
  private val cameras: List<VisionEstimator> = mutableListOf(),
  private val field: Field2d
) : SubsystemBase(), HolonomicDrive {

  /** The kinematics that convert [ChassisSpeeds] into multiple [SwerveModuleState] objects. */
  private val kinematics = SwerveDriveKinematics(
    *this.modules.map { it.location }.toTypedArray()
  )

  /** The current speed of the robot's drive. */
  var currentSpeeds = ChassisSpeeds()

  /** Current estimated vision pose */
  var visionPose = Pose2d()

  /** Pose estimator that estimates the robot's position as a [Pose2d]. */
  private val poseEstimator = SwerveDrivePoseEstimator(
    kinematics,
    ahrs.heading,
    getPositions(),
    RobotConstants.INITIAL_POSE,
    VisionConstants.ENCODER_TRUST,
    VisionConstants.VISION_TRUST
  )

  var desiredSpeeds: ChassisSpeeds = ChassisSpeeds()

  private var maxSpeed: Double = 0.0

  override fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = desiredSpeeds

    // Converts the desired [ChassisSpeeds] into an array of [SwerveModuleState].
    val desiredModuleStates =
      this.kinematics.toSwerveModuleStates(this.desiredSpeeds)

    // Scale down module speed if a module is going faster than the max speed, and prevent early desaturation.
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredModuleStates,
      SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
    )

    for (i in this.modules.indices) {
      this.modules[i].state = desiredModuleStates[i]
    }

    for (module in modules)
      module.update()
  }

  fun setVoltage(volts: Double) {
    modules.forEach {
      it.setVoltage(volts)
    }
  }

  fun getModuleVel(): Double {
    var totalVel = 0.0
    modules.forEach { totalVel += it.state.speedMetersPerSecond }
    return totalVel / modules.size
  }

  /** The measured pitch of the robot from the gyro sensor. */
  val pitch: Rotation2d
    get() = Rotation2d(MathUtil.angleModulus(ahrs.pitch.radians))

  /** The measured roll of the robot from the gyro sensor. */
  val roll: Rotation2d
    get() = Rotation2d(MathUtil.angleModulus(ahrs.roll.radians))

  /** The (x, y, theta) position of the robot on the field. */
  override var pose: Pose2d
    get() = this.poseEstimator.estimatedPosition
    set(value) {
      this.poseEstimator.resetPosition(
        ahrs.heading,
        getPositions(),
        value
      )
    }

  override fun periodic() {
    // Updates the robot's currentSpeeds.
    currentSpeeds = kinematics.toChassisSpeeds(
      modules[0].state,
      modules[1].state,
      modules[2].state,
      modules[3].state
    )

    val transVel = hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
    if (transVel > maxSpeed) maxSpeed = transVel

    if (cameras.isNotEmpty()) localize()

    // Update the robot's pose using the gyro heading and the SwerveModulePositions of each module.
    this.poseEstimator.update(
      ahrs.heading,
      getPositions()
    )

    // Sets the robot's pose and individual module rotations on the SmartDashboard [Field2d] widget.
    setRobotPose()
  }

  /** Stops the robot's drive. */
  override fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  /** @return An array of [SwerveModulePosition] for each module, containing distance and angle. */
  private fun getPositions(): Array<SwerveModulePosition> {
    return Array(modules.size) { i -> modules[i].position }
  }

  /** @return An array of [SwerveModuleState] for each module, containing speed and angle. */
  private fun getStates(): Array<SwerveModuleState> {
    return Array(modules.size) { i -> modules[i].state }
  }

  private fun setRobotPose() {
    this.field.robotPose = this.pose
    this.field.getObject("FL").pose = this.pose.plus(Transform2d(Translation2d(SwerveConstants.WHEELBASE / 2, SwerveConstants.TRACKWIDTH / 2), this.getPositions()[0].angle))
    this.field.getObject("FR").pose = this.pose.plus(Transform2d(Translation2d(SwerveConstants.WHEELBASE / 2, -SwerveConstants.TRACKWIDTH / 2), this.getPositions()[1].angle))
    this.field.getObject("BL").pose = this.pose.plus(Transform2d(Translation2d(-SwerveConstants.WHEELBASE / 2, SwerveConstants.TRACKWIDTH / 2), this.getPositions()[2].angle))
    this.field.getObject("BR").pose = this.pose.plus(Transform2d(Translation2d(-SwerveConstants.WHEELBASE / 2, -SwerveConstants.TRACKWIDTH / 2), this.getPositions()[0].angle))
  }

  private fun localize() = try {
    for (camera in cameras) {
      val result = camera.estimatedPose(pose.rotation)
      if (result.isPresent) {
        val presentResult = result.get()
        val numTargets = presentResult.targetsUsed.size
        var tagDistance = 0.0
        var avgAmbiguity = 0.0

        for (tag in presentResult.targetsUsed) {
          val tagPose = camera.fieldTags.getTagPose(tag.fiducialId)
          if (tagPose.isPresent) {
            val estimatedToTag = presentResult.estimatedPose.minus(tagPose.get())
            tagDistance += sqrt(estimatedToTag.x.pow(2) + estimatedToTag.y.pow(2)) / numTargets
            avgAmbiguity = tag.poseAmbiguity / numTargets
          } else {
            tagDistance = Double.MAX_VALUE
            avgAmbiguity = Double.MAX_VALUE
            break
          }
        }

        if (presentResult.timestampSeconds > 0 &&
          avgAmbiguity <= VisionConstants.MAX_AMBIGUITY &&
          numTargets < 2 && tagDistance <= VisionConstants.MAX_DISTANCE_SINGLE_TAG ||
          numTargets >= 2 && tagDistance <= VisionConstants.MAX_DISTANCE_MULTI_TAG
        ) {
//          poseEstimator.addVisionMeasurement(
//            presentResult.estimatedPose.toPose2d(),
//            presentResult.timestampSeconds
//          )
          visionPose = presentResult.estimatedPose.toPose2d()
        }
      }
    }
  } catch (e: Error) {
    DriverStation.reportError(
      "!!!!!!!!! VISION ERROR !!!!!!!",
      e.stackTrace
    )
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Poses and ChassisSpeeds")
    builder.addDoubleArrayProperty("1.1 Estimated Pose", { doubleArrayOf(pose.x, pose.y, pose.rotation.radians) }, null)
    builder.addDoubleArrayProperty("1.2 Vision Pose", { doubleArrayOf(visionPose.x, visionPose.y, visionPose.rotation.radians) }, null)
    builder.addDoubleArrayProperty("1.3 Current Chassis Speeds", { doubleArrayOf(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond, currentSpeeds.omegaRadiansPerSecond) }, null)
    builder.addDoubleArrayProperty("1.4 Desired Chassis Speeds", { doubleArrayOf(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond) }, null)
    builder.addDoubleProperty("1.5 Max Recorded Speed", { maxSpeed }, null)

    builder.publishConstString("2.0", "Steering Rot (Std Order FL, FR, BL, BR)")
    builder.addDoubleArrayProperty(
      "2.1 Current Rotation",
      { DoubleArray(modules.size) { index -> modules[index].position.angle.rotations } },
      null)
    builder.addDoubleArrayProperty("2.2 Desired Rotation",
      { DoubleArray(modules.size) { index -> modules[index].desiredState.angle.rotations } },
      null)

    builder.publishConstString("3.0", "Module Driving Speeds (Std Order FL, FR, BL, BR)")
    builder.addDoubleArrayProperty(
      "3.1 Desired Speed",
      { DoubleArray(modules.size) { index -> modules[index].desiredState.speedMetersPerSecond } },
      null
    )
    builder.addDoubleArrayProperty(
      "3.2 Current Speeds",
      { DoubleArray(modules.size) { index -> modules[index].state.speedMetersPerSecond } },
      null
    )

    builder.publishConstString("4.0", "Last Module Voltages (Standard Order, FL, FR, BL, BR)")
    builder.addDoubleArrayProperty(
      "4.1 Driving",
      { DoubleArray(modules.size) { index -> modules[index].lastDrivingVoltage() } },
      null
    )
    builder.addDoubleArrayProperty(
      "4.2 Steering",
      { DoubleArray(modules.size) { index -> modules[index].lastSteeringVoltage() } },
      null
    )

    builder.publishConstString("5.0", "AHRS Values")
    builder.addDoubleProperty("5.1 Heading Degrees", { ahrs.heading.degrees }, null)
    builder.addDoubleProperty("5.2 Pitch Degrees", { ahrs.pitch.degrees }, null)
    builder.addDoubleProperty("5.3 Roll Degrees", { ahrs.roll.degrees }, null)
    builder.addDoubleProperty("5.4 Angular X Vel", { ahrs.angularXVel() }, null)

    // Note: You should also tune UPR too
    builder.publishConstString("6.0", "Tuning Values")
    builder.addDoubleProperty("6.1 FL Drive P", { modules[0].driveController.p }, { value -> modules[0].driveController.p = value })
    builder.addDoubleProperty("6.2 FL Drive D", { modules[0].driveController.d }, { value -> modules[0].driveController.d = value })
    builder.addDoubleProperty("6.3 FL Turn P", { modules[0].turnController.p }, { value -> modules[0].turnController.p = value })
    builder.addDoubleProperty("6.4 FL Turn D", { modules[0].turnController.d }, { value -> modules[0].turnController.d = value })
    builder.addDoubleProperty("6.5 FR Drive P", { modules[1].driveController.p }, { value -> modules[1].driveController.p = value })
    builder.addDoubleProperty("6.6 FR Drive D", { modules[1].driveController.d }, { value -> modules[1].driveController.d = value })
    builder.addDoubleProperty("6.8 FR Turn P", { modules[1].turnController.p }, { value -> modules[1].turnController.p = value })
    builder.addDoubleProperty("6.9 FR Turn D", { modules[1].turnController.d }, { value -> modules[1].turnController.d = value })
    builder.addDoubleProperty("6.10 BL Drive P", { modules[2].driveController.p }, { value -> modules[2].driveController.p = value })
    builder.addDoubleProperty("6.11 BL Drive D", { modules[2].driveController.d }, { value -> modules[2].driveController.d = value })
    builder.addDoubleProperty("6.12 BL Turn P", { modules[2].turnController.p }, { value -> modules[2].turnController.p = value })
    builder.addDoubleProperty("6.13 BL Turn D", { modules[2].turnController.d }, { value -> modules[2].turnController.d = value })
    builder.addDoubleProperty("6.14 BR Drive P", { modules[3].driveController.p }, { value -> modules[3].driveController.p = value })
    builder.addDoubleProperty("6.15 BR Drive D", { modules[3].driveController.d }, { value -> modules[3].driveController.d = value })
    builder.addDoubleProperty("6.16 BR Turn P", { modules[3].turnController.p }, { value -> modules[3].turnController.p = value })
    builder.addDoubleProperty("6.17 BR Turn D", { modules[3].turnController.d }, { value -> modules[3].turnController.d = value })
  }

  companion object {
    /** Create a [SwerveDrive] using [SwerveConstants]. */
    fun createSwerve(ahrs: AHRS, field: Field2d): SwerveDrive {
      val driveMotorController = { PIDController(SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD) }
      val turnMotorController = { PIDController(SwerveConstants.TURN_KP, SwerveConstants.TURN_KI, SwerveConstants.TURN_KD) }
      val driveFeedforward = SimpleMotorFeedforward(SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA)
      val modules = listOf(
        SwerveModule.create(
          "FLModule",
          makeDrivingMotor(
            "FL",
            SwerveConstants.DRIVE_MOTOR_FL,
            inverted = false
          ),
          makeTurningMotor(
            "FL",
            SwerveConstants.TURN_MOTOR_FL,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_FL,
            SwerveConstants.TURN_ENC_OFFSET_FL
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(SwerveConstants.WHEELBASE / 2, SwerveConstants.TRACKWIDTH / 2)
        ),
        SwerveModule.create(
          "FRModule",
          makeDrivingMotor(
            "FR",
            SwerveConstants.DRIVE_MOTOR_FR,
            inverted = false
          ),
          makeTurningMotor(
            "FR",
            SwerveConstants.TURN_MOTOR_FR,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_FR,
            SwerveConstants.TURN_ENC_OFFSET_FR
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(SwerveConstants.WHEELBASE / 2, -SwerveConstants.TRACKWIDTH / 2)
        ),
        SwerveModule.create(
          "BLModule",
          makeDrivingMotor(
            "BL",
            SwerveConstants.DRIVE_MOTOR_BL,
            inverted = false
          ),
          makeTurningMotor(
            "BL",
            SwerveConstants.TURN_MOTOR_BL,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_BL,
            SwerveConstants.TURN_ENC_OFFSET_BL
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(-SwerveConstants.WHEELBASE / 2, SwerveConstants.TRACKWIDTH / 2)
        ),
        SwerveModule.create(
          "BRModule",
          makeDrivingMotor(
            "BR",
            SwerveConstants.DRIVE_MOTOR_BR,
            inverted = false
          ),
          makeTurningMotor(
            "BR",
            SwerveConstants.TURN_MOTOR_BR,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_BR,
            SwerveConstants.TURN_ENC_OFFSET_BR
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(-SwerveConstants.WHEELBASE / 2, -SwerveConstants.TRACKWIDTH / 2)
        )
      )
      return if (isReal()) {
        SwerveDrive(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          VisionConstants.ESTIMATORS,
          field
        )
      } else {
        SwerveSim(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          VisionConstants.ESTIMATORS,
          field
        )
      }
    }

    /** Helper to make turning motors for swerve. */
    private fun makeDrivingMotor(
      name: String,
      motorId: Int,
      inverted: Boolean
    ) =
      createSparkMax(
        name = name + "Drive",
        id = motorId,
        enableBrakeMode = true,
        inverted = inverted,
        encCreator =
        NEOEncoder.creator(
          SwerveConstants.DRIVE_UPR,
          SwerveConstants.DRIVE_GEARING
        ),
        currentLimit = SwerveConstants.DRIVE_CURRENT_LIM
      )

    /** Helper to make turning motors for swerve. */
    private fun makeTurningMotor(
      name: String,
      motorId: Int,
      inverted: Boolean,
      sensorPhase: Boolean,
      encoderChannel: Int,
      offset: Double
    ) =
      createSparkMax(
        name = name + "Turn",
        id = motorId,
        enableBrakeMode = false,
        inverted = inverted,
        encCreator = AbsoluteEncoder.creator(
          encoderChannel,
          offset,
          SwerveConstants.TURN_UPR,
          sensorPhase
        ),
        currentLimit = SwerveConstants.STEERING_CURRENT_LIM
      )
  }
}
