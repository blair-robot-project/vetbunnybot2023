package frc.team449.system

import com.revrobotics.*
import edu.wpi.first.wpilibj.RobotController

object SparkUtil {
  fun applySparkSettings(

    // spark max information
    sparkMax: CANSparkMax,
    enableBrakeMode: Boolean = true,
    inverted: Boolean = false,
    currentLimit: Int = 0,
    enableVoltageComp: Boolean = false,
    slaveSparks: Map<Int, Boolean> = mapOf(),
    controlFrameRateMillis: Int = -1,
    statusFrameRatesMillis: Map<CANSparkMaxLowLevel.PeriodicFrame, Int> = mapOf(),

    // encoder information
    encoder: MotorFeedbackSensor,
    unitPerRotation: Double,
    gearing: Double,
    offset: Double = 0.0,
    encInverted: Boolean = false
  ) {
    sparkMax.restoreFactoryDefaults()
    sparkMax.idleMode = if (enableBrakeMode) CANSparkMax.IdleMode.kBrake else CANSparkMax.IdleMode.kCoast
    sparkMax.inverted = inverted
    if (currentLimit > 0) sparkMax.setSmartCurrentLimit(currentLimit)
    if (enableVoltageComp) sparkMax.enableVoltageCompensation(RobotController.getBatteryVoltage()) else sparkMax.disableVoltageCompensation()
    if (controlFrameRateMillis >= 1) sparkMax.setControlFramePeriodMs(controlFrameRateMillis) // Must be between 1 and 100 ms.
    for ((statusFrame, period) in statusFrameRatesMillis) {
      sparkMax.setPeriodicFramePeriod(statusFrame, period)
    }

    for ((slavePort, slaveInverted) in slaveSparks) {
      val slave = CANSparkMax(slavePort, CANSparkMaxLowLevel.MotorType.kBrushless)
      slave.restoreFactoryDefaults()
      slave.follow(sparkMax, slaveInverted)
      slave.idleMode = sparkMax.idleMode
      if (currentLimit > 0) slave.setSmartCurrentLimit(currentLimit)
      slave.burnFlash()
    }

    when (encoder) {
      is SparkMaxAbsoluteEncoder -> {
        encoder.positionConversionFactor = unitPerRotation * gearing
        encoder.velocityConversionFactor = unitPerRotation * gearing / 60
        encoder.zeroOffset = offset
        encoder.setInverted(encInverted)
      }

      is SparkMaxRelativeEncoder -> {
        encoder.positionConversionFactor = unitPerRotation * gearing
        encoder.velocityConversionFactor = unitPerRotation * gearing / 60
      }

      is SparkMaxAlternateEncoder -> {
        encoder.positionConversionFactor = unitPerRotation * gearing
        encoder.velocityConversionFactor = unitPerRotation * gearing / 60
        encoder.setInverted(encInverted)
      }

      else -> throw IllegalStateException("UNSUPPORTED ENCODER PLUGGED INTO SPARK MAX.")
    }

    sparkMax.pidController.setFeedbackDevice(encoder)

    sparkMax.burnFlash()
  }

  fun enableContinuousInput(sparkMax: CANSparkMax, min: Double, max: Double) {
    val controller = sparkMax.pidController

    controller.setPositionPIDWrappingEnabled(true)
    controller.setPositionPIDWrappingMinInput(min)
    controller.setPositionPIDWrappingMaxInput(max)
  }
}
