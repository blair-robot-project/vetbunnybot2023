// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.team449.robot2023.commands.characterization

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import java.util.LinkedList
import java.util.function.BiConsumer
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.math.abs

class Characterization : Command {
  private val forwards: Boolean
  private val isDrive: Boolean
  private val dataPrimary: FeedForwardCharacterizationData
  private val dataSecondary: FeedForwardCharacterizationData?
  private val voltageConsumerSimple: Consumer<Double>?
  private val voltageConsumerDrive: BiConsumer<Double, Double>?
  private val velocitySupplierPrimary: Supplier<Double>
  private val velocitySupplierSecondary: Supplier<Double>?
  private val timer = Timer()

  /** Creates a new FeedForwardCharacterization for a differential drive.  */
  constructor(
    drive: Subsystem?,
    forwards: Boolean,
    leftData: FeedForwardCharacterizationData,
    rightData: FeedForwardCharacterizationData?,
    voltageConsumer: BiConsumer<Double, Double>?,
    leftVelocitySupplier: Supplier<Double>,
    rightVelocitySupplier: Supplier<Double>?
  ) {
    addRequirements(drive)
    this.forwards = forwards
    isDrive = true
    dataPrimary = leftData
    dataSecondary = rightData
    voltageConsumerSimple = null
    voltageConsumerDrive = voltageConsumer
    velocitySupplierPrimary = leftVelocitySupplier
    velocitySupplierSecondary = rightVelocitySupplier
  }

  /** Creates a new FeedForwardCharacterization for a simple subsystem.  */
  constructor(
    subsystem: Subsystem?,
    forwards: Boolean,
    data: FeedForwardCharacterizationData,
    voltageConsumer: Consumer<Double>?,
    velocitySupplier: Supplier<Double>
  ) {
    addRequirements(subsystem)
    this.forwards = forwards
    isDrive = false
    dataPrimary = data
    dataSecondary = null
    voltageConsumerSimple = voltageConsumer
    voltageConsumerDrive = null
    velocitySupplierPrimary = velocitySupplier
    velocitySupplierSecondary = null
  }

  // Called when the command is initially scheduled.
  override fun initialize() {
    timer.reset()
    timer.start()
  }

  // Called every time the scheduler runs while the command is scheduled.
  override fun execute() {
    if (timer.get() < startDelaySecs) {
      if (isDrive) {
        voltageConsumerDrive!!.accept(0.0, 0.0)
      } else {
        voltageConsumerSimple!!.accept(0.0)
      }
    } else {
      val voltage = (timer.get() - startDelaySecs) * rampRateVoltsPerSec * if (forwards) 1 else -1
      if (isDrive) {
        voltageConsumerDrive!!.accept(voltage, voltage)
      } else {
        voltageConsumerSimple!!.accept(voltage)
      }
      dataPrimary.add(velocitySupplierPrimary.get(), voltage)
      if (isDrive) {
        dataSecondary!!.add(velocitySupplierSecondary!!.get(), voltage)
      }
    }
  }

  // Called once the command ends or is interrupted.
  override fun end(interrupted: Boolean) {
    if (isDrive) {
      voltageConsumerDrive!!.accept(0.0, 0.0)
    } else {
      voltageConsumerSimple!!.accept(0.0)
    }
    timer.stop()
    dataPrimary.print()
    if (isDrive) {
      dataSecondary!!.print()
    }
  }

  // Returns true when the command should end.
  override fun isFinished(): Boolean {
    return false
  }

  class FeedForwardCharacterizationData(private val name: String) {
    private val velocityData: MutableList<Double> = LinkedList()
    private val voltageData: MutableList<Double> = LinkedList()
    fun add(velocity: Double, voltage: Double) {
      if (abs(velocity) > 1E-4) {
        velocityData.add(abs(velocity))
        voltageData.add(abs(voltage))
      }
    }

    fun print() {
      if (velocityData.size == 0 || voltageData.size == 0) {
        return
      }
      val regression = Regression(
        velocityData.stream().mapToDouble { obj: Double -> obj }.toArray(),
        voltageData.stream().mapToDouble { obj: Double -> obj }.toArray(),
        1
      )
      println("FF Characterization Results ($name):")
      println("\tCount=" + velocityData.size.toString() + "")
      println(java.lang.String.format("\tR2=%.5f", regression.R2()))
      println(java.lang.String.format("\tkS=%.5f", regression.beta(0)))
      println(java.lang.String.format("\tkV=%.5f", regression.beta(1)))
    }
  }

  companion object {
    private const val startDelaySecs = 2.0
    private const val rampRateVoltsPerSec = 0.05
  }
}