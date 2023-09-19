package frc.team449.robot2023.subsystems.arm

import frc.team449.robot2023.subsystems.arm.control.ArmTrajectory

object ArmPaths {

  // scoring trajs
//  var MID_STOW = ArmTrajectory("Mid_Stow.json")
//  var STOW_MID = ArmTrajectory("Stow_Mid.json")
//  var HIGH_STOW = ArmTrajectory("High_Stow.json")
//  var STOW_HIGH = ArmTrajectory("Stow_High.json")
//  var GROUND_STOW = ArmTrajectory("Ground_Stow.json")
//  var STOW_GROUND = ArmTrajectory("Stow_Ground.json")

  val stowSingle = ArmTrajectory("stowSingle.json")
  val singleStow = ArmTrajectory("singleStow.json")

  // todo: get position for double substation and create trajectory. put trajectory into stowDouble and doubleStow

  val stowDouble = ArmTrajectory("stowDouble.json")
  val doubleStow = ArmTrajectory("doubleStow.json")

  val stowCone = ArmTrajectory("stowCone.json")
  val coneStow = ArmTrajectory("coneStow.json")
  val stowCube = ArmTrajectory("stowCube.json")
  val cubeStow = ArmTrajectory("cubeStow.json")

  val stowMid = ArmTrajectory("stowMid.json")
  val midStow = ArmTrajectory("midStow.json")

  val stowHigh = ArmTrajectory("stowHigh.json")
  val highStow = ArmTrajectory("highStow.json")

  val highMid = ArmTrajectory("highMid.json")
  val midHigh = ArmTrajectory("midHigh.json")

  val cubeCone = ArmTrajectory("cubeCone.json")
  val coneCube = ArmTrajectory("coneCube.json")

  /** Backstow Trajectories */

  val backHigh = ArmTrajectory("backHigh.json")
  val highBack = ArmTrajectory("highBack.json")

  val backDouble = ArmTrajectory("backDouble.json")
  val doubleBack = ArmTrajectory("doubleBack.json")

  val backMid = ArmTrajectory("backMid.json")
  val midBack = ArmTrajectory("midBack.json")

  val backStow = ArmTrajectory("backStow.json")
  val stowBack = ArmTrajectory("stowBack.json")

  val backSingle = ArmTrajectory("backSingle.json")
  val singleBack = ArmTrajectory("singleBack.json")

  val backCube = ArmTrajectory("backCube.json")
  val cubeBack = ArmTrajectory("cubeBack.json")

  val backCone = ArmTrajectory("backCone.json")
  val coneBack = ArmTrajectory("coneBack.json")

  val backLow = ArmTrajectory("backLow.json")
  val lowBack = ArmTrajectory("lowBack.json")
  val stowLow = ArmTrajectory("stowLow.json")
  val lowStow = ArmTrajectory("lowStow.json")
  val lowMid = ArmTrajectory("lowMid.json")
  val lowHigh = ArmTrajectory("lowHigh.json")
  val midLow = ArmTrajectory("midLow.json")
  val highLow = ArmTrajectory("highLow.json")

  /** Auto Specific trajectories */

  val coneHigh = ArmTrajectory("coneHigh.json")
  val highCone = ArmTrajectory("highCone.json")
  val cubeMid = ArmTrajectory("cubeMid.json")
  val cubeHigh = ArmTrajectory("cubeHigh.json")
  val highCube = ArmTrajectory("highCube.json")

  val trajectories = listOf(
    backLow,
    lowBack,
    stowLow,
    lowStow,
    stowSingle,
    singleStow,
    stowDouble,
    doubleStow,
    stowCone,
    coneStow,
    stowCube,
    cubeStow,
    stowMid,
    midStow,
    stowHigh,
    highStow,
    coneHigh,
    highCone,
    cubeHigh,
    highCube,
    midHigh,
    highMid,
    cubeCone,
    coneCube,
    cubeMid,
    backStow,
    stowBack,
    backDouble,
    doubleBack,
    backHigh,
    highBack,
    backMid,
    midBack,
    backSingle,
    singleBack,
    coneBack,
    backCone,
    cubeBack,
    backCube,
    lowMid,
    lowHigh,
    highLow,
    midLow
  )

  fun parseTrajectories() {
    trajectories.forEach {
      it.parse()
    }
  }
}
