package frc.team449.robot2023.subsystems.nodeTracker

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.Robot
import frc.team449.robot2023.commands.autoScore.GridAlign
import frc.team449.robot2023.constants.field.FieldConstants

class ObjectiveTracker(
  selectorIO: NodeSelectorIO,
  robot: Robot
) : SubsystemBase() {
  private val selectorIO: NodeSelectorIO
  private val selectorInputs: NodeSelectorIO.NodeSelectorIOInputs = NodeSelectorIO.NodeSelectorIOInputs()
  private val objective = Objective()
  private val prevObjective = Objective()

  class Objective(
    var nodeRow: Int = 0,
    var nodeLevel: GridAlign.Levels = GridAlign.Levels.LOW
  ) {
    val isConeNode: Boolean
      get() = (
        nodeLevel != GridAlign.Levels.LOW &&
          (nodeRow == 0 || nodeRow == 2 || nodeRow == 3 || nodeRow == 5 || nodeRow == 6 || nodeRow == 8)
        )
  }

  private val alignGen = GridAlign(robot)
  private val rows = arrayOf(
    FieldConstants.TargetPosition.Position1,
    FieldConstants.TargetPosition.Position2,
    FieldConstants.TargetPosition.Position3,
    FieldConstants.TargetPosition.Position4,
    FieldConstants.TargetPosition.Position5,
    FieldConstants.TargetPosition.Position6,
    FieldConstants.TargetPosition.Position7,
    FieldConstants.TargetPosition.Position8,
    FieldConstants.TargetPosition.Position9
  )

  init {
    println("[Init] Creating ObjectiveTracker")
    this.selectorIO = selectorIO
  }

  fun updateObj(inputs: NodeSelectorIO.NodeSelectorIOInputs, obj: Objective) {
    if (inputs.selectedNode != -1) {
      if (DriverStation.getAlliance() == Alliance.Blue) {
        obj.nodeRow = 8 - inputs.selectedNode % 9
      } else {
        obj.nodeRow = inputs.selectedNode % 9
      }
      if (inputs.selectedNode < 9) {
        obj.nodeLevel = GridAlign.Levels.LOW
      } else if (inputs.selectedNode < 18) {
        obj.nodeLevel = GridAlign.Levels.MID
      } else {
        obj.nodeLevel = GridAlign.Levels.HIGH
      }
      inputs.selectedNode = -1
    }
  }

  override fun periodic() {
    selectorIO.updateInputs(selectorInputs)
    updateObj(selectorInputs, objective)

    if (objective.nodeRow != prevObjective.nodeRow ||
      objective.nodeLevel != prevObjective.nodeLevel &&
      !DriverStation.isAutonomous()
    ) {
      alignGen.autoScore(
        rows[objective.nodeRow],
        DriverStation.getAlliance().equals(Alliance.Red),
        objective.nodeLevel,
        objective.isConeNode
      ).schedule()
    }

    prevObjective.nodeRow = objective.nodeRow
    prevObjective.nodeLevel = objective.nodeLevel
  }

  companion object {
    fun createObjTracker(robot: Robot): ObjectiveTracker {
      return ObjectiveTracker(
        NodeSelectorIOServer(),
        robot
      )
    }
  }
}
