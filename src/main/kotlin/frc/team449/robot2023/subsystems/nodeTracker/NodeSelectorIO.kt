package frc.team449.robot2023.subsystems.nodeTracker


interface NodeSelectorIO {
  class NodeSelectorIOInputs {
    var selectedNode: Int = -1
  }

  fun updateInputs(inputs: NodeSelectorIOInputs) {}
  fun setSelected(selected: Long) {}
}