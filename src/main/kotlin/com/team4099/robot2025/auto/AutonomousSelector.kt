package com.team4099.robot2023.auto

import com.team4099.robot2023.auto.mode.ExamplePathAuto
import com.team4099.robot2023.auto.mode.FiveNoteAutoPath
import com.team4099.robot2023.auto.mode.FourNoteAutoPath
import com.team4099.robot2023.auto.mode.FourNoteAutoPathWithFirstCenterSide
import com.team4099.robot2023.auto.mode.FourNoteAutoPathWithFirstSourceSide
import com.team4099.robot2023.auto.mode.FourNoteLeftCenterLine
import com.team4099.robot2023.auto.mode.FourNoteMiddleCenterLine
import com.team4099.robot2023.auto.mode.FourNoteRightCenterLine
import com.team4099.robot2023.auto.mode.PreloadAndLeaveCenterSubwooferAutoPath
import com.team4099.robot2023.auto.mode.PreloadAndLeaveFromAmpSubwooferAutoPath
import com.team4099.robot2023.auto.mode.PreloadAndLeaveFromSourceSubwooferAutoPath
import com.team4099.robot2023.auto.mode.SixNoteAutoPath
import com.team4099.robot2023.auto.mode.TestAutoPath
import com.team4099.robot2023.auto.mode.ThreeNoteAndPickupCenterlineSourceAutoPath
import com.team4099.robot2023.auto.mode.ThreeNoteAndPickupCenterlineWithoutFirstNoteSourceAutoPath
import com.team4099.robot2023.auto.mode.ThreeNoteCenterlineFromAmpAutoPath
import com.team4099.robot2023.auto.mode.ThreeNoteCenterlineWithoutFirstNoteAmpAutoPath
import com.team4099.robot2023.auto.mode.TwoNoteCenterlineFromAmpAutoPath
import com.team4099.robot2023.auto.mode.TwoNoteCenterlineFromSourceAutoPath
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.util.AllianceFlipUtil
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds

object AutonomousSelector {
  //  private var orientationChooser: SendableChooser<Angle> = SendableChooser()
  private var autonomousModeChooser: LoggedDashboardChooser<AutonomousMode> =
    LoggedDashboardChooser("AutonomousMode")
  private var waitBeforeCommandSlider: GenericEntry
  private var secondaryWaitInAuto: GenericEntry

  init {
    val autoTab = Shuffleboard.getTab("Pre-match")
    //    orientationChooser.setDefaultOption("Forward", 0.degrees)
    //    orientationChooser.addOption("Backwards", 180.degrees)
    //    orientationChooser.addOption("Left", 90.degrees)
    //    orientationChooser.addOption("Right", 270.degrees)
    //    autoTab.add("Starting Orientation", orientationChooser)

    autonomousModeChooser.addOption(
      "Four Note Wing Auto (Amp Side Note First, Default)",
      AutonomousMode.FOUR_NOTE_AUTO_PATH_WITH_AMP_SIDE_FIRST
    )
    autonomousModeChooser.addOption(
      "Four Note Wing Auto (Center Wing Note First)",
      AutonomousMode.FOUR_NOTE_AUTO_PATH_WITH_CENTER_SIDE_FIRST
    )
    autonomousModeChooser.addOption(
      "Four Note Wing Auto (Source Side Note First)",
      AutonomousMode.FOUR_NOTE_AUTO_PATH_WITH_SOURCE_SIDE_FIRST
    )

    /*

    autonomousModeChooser.addOption(
      "Four Note Right Auto(1 Wing + 2 Centerline)", AutonomousMode.FOUR_NOTE_RIGHT_AUTO_PATH
    )
    autonomousModeChooser.addOption(
      "Four Note Middle Auto(1 Wing + 2 Centerline)", AutonomousMode.FOUR_NOTE_MIDDLE_AUTO_PATH
    )
    autonomousModeChooser.addOption(
      "Four Note LEFT Auto(1 Wing + 2 Centerline)", AutonomousMode.FOUR_NOTE_LEFT_AUTO_PATH
    )

     */

    autonomousModeChooser.addOption(
      "Five Note Auto from Center Subwoofer", AutonomousMode.FIVE_NOTE_AUTO_PATH
    )

    /*
    autonomousModeChooser.addOption(
      "Two Note Centerline Auto from Source Side of Subwoofer",
      AutonomousMode.TWO_NOTE_CENTERLINE_FROM_SOURCE
    )

    autonomousModeChooser.addOption(
      "Two Note Centerline Auto from Amp Side of Subwoofer",
      AutonomousMode.TWO_NOTE_CENTERLINE_FROM_AMP
    )

     */

    autonomousModeChooser.addOption(
      "Three Note + Pickup Centerline Auto from Amp Side of Subwoofer",
      AutonomousMode.THREE_NOTE_CENTERLINE_FROM_AMP
    )

    autonomousModeChooser.addOption(
      "Three Note + Pickup Centerline WITHOUT First Note Auto from Amp Side of Subwoofer",
      AutonomousMode.THREE_NOTE_CENTERLINE_WITHOUT_FIRST_NOTE_FROM_AMP
    )

    autonomousModeChooser.addOption(
      "Three Note + Pickup Centerline Auto from Source Side of Subwoofer",
      AutonomousMode.THREE_NOTE_AND_PICKUP_CENTERLINE_FROM_SOURCE
    )

    autonomousModeChooser.addOption(
      "Three Note + Pickup Centerline WITHOUT First Note Auto from Source Side of Subwoofer",
      AutonomousMode.THREE_NOTE_AND_PICKUP_CENTERLINE_WITHOUT_FIRST_NOTE_FROM_SOURCE
    )

    autonomousModeChooser.addOption(
      "Preload + Leave from Amp Side of Subwoofer",
      AutonomousMode.PRELOAD_AND_LEAVE_LEFT_SUBWOOFER
    )
    autonomousModeChooser.addOption(
      "Preload + Leave from Source Side of Subwoofer",
      AutonomousMode.PRELOAD_AND_LEAVE_RIGHT_SUBWOOFER
    )
    autonomousModeChooser.addOption(
      "Preload + Leave from Center Side of Subwoofer",
      AutonomousMode.PRELOAD_AND_LEAVE_CENTER_SUBWOOFER
    )

    /*

    autonomousModeChooser.addOption("Six Note Path", AutonomousMode.SIX_NOTE_AUTO_PATH)
    autonomousModeChooser.addOption(
      "Six Note Path with Pickup", AutonomousMode.SIX_NOTE_WITH_PICKUP_PATH
    )

     */
    autonomousModeChooser.addOption("Test Auto Path", AutonomousMode.TEST_AUTO_PATH)
    // autonomousModeChooser.addOption("Characterize Elevator",
    // AutonomousMode.ELEVATOR_CHARACTERIZE)
    autoTab.add("Mode", autonomousModeChooser.sendableChooser).withSize(4, 2).withPosition(2, 0)

    waitBeforeCommandSlider =
      autoTab
        .add("Wait Time Before Shooting", 0)
        .withSize(3, 2)
        .withPosition(0, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .entry
    secondaryWaitInAuto =
      autoTab
        .add("Secondary Wait Time Between Shooting and Driving", 0)
        .withSize(3, 2)
        .withPosition(3, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .entry
  }

  val waitTime: Time
    get() = waitBeforeCommandSlider.getDouble(0.0).seconds

  val secondaryWaitTime: Time
    get() = secondaryWaitInAuto.getDouble(0.0).seconds

  fun getCommand(drivetrain: Drivetrain, superstructure: Superstructure): Command {
    val mode = autonomousModeChooser.get()

    when (mode) {
      AutonomousMode.TEST_AUTO_PATH ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.tempZeroGyroYaw(TestAutoPath.startingPose.rotation)
            drivetrain.resetFieldFrameEstimator(
              AllianceFlipUtil.apply(TestAutoPath.startingPose)
            )
          })
          .andThen(TestAutoPath(drivetrain, superstructure))
      AutonomousMode.SIX_NOTE_AUTO_PATH ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.tempZeroGyroYaw(TestAutoPath.startingPose.rotation)
            drivetrain.resetFieldFrameEstimator(
              AllianceFlipUtil.apply(TestAutoPath.startingPose)
            )
          })
          .andThen(SixNoteAutoPath(drivetrain, superstructure))
      AutonomousMode.FOUR_NOTE_AUTO_PATH_WITH_AMP_SIDE_FIRST ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose = AllianceFlipUtil.apply(FourNoteAutoPath.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(FourNoteAutoPath(drivetrain, superstructure))
      AutonomousMode.FOUR_NOTE_AUTO_PATH_WITH_CENTER_SIDE_FIRST ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose =
              AllianceFlipUtil.apply(FourNoteAutoPathWithFirstCenterSide.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(FourNoteAutoPathWithFirstCenterSide(drivetrain, superstructure))
      AutonomousMode.FOUR_NOTE_AUTO_PATH_WITH_SOURCE_SIDE_FIRST ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose =
              AllianceFlipUtil.apply(FourNoteAutoPathWithFirstSourceSide.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(FourNoteAutoPathWithFirstSourceSide(drivetrain, superstructure))
      AutonomousMode.FOUR_NOTE_RIGHT_AUTO_PATH ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose = AllianceFlipUtil.apply(FourNoteRightCenterLine.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(FourNoteRightCenterLine(drivetrain, superstructure))
      AutonomousMode.FOUR_NOTE_MIDDLE_AUTO_PATH ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose = AllianceFlipUtil.apply(FourNoteMiddleCenterLine.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(FourNoteMiddleCenterLine(drivetrain, superstructure))
      AutonomousMode.FOUR_NOTE_LEFT_AUTO_PATH ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose = AllianceFlipUtil.apply(FourNoteLeftCenterLine.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(FourNoteMiddleCenterLine(drivetrain, superstructure))
      AutonomousMode.FIVE_NOTE_AUTO_PATH ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose = AllianceFlipUtil.apply(FiveNoteAutoPath.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(FiveNoteAutoPath(drivetrain, superstructure))
      AutonomousMode.TWO_NOTE_CENTERLINE_FROM_SOURCE ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose =
              AllianceFlipUtil.apply(TwoNoteCenterlineFromSourceAutoPath.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(TwoNoteCenterlineFromSourceAutoPath(drivetrain, superstructure))
      AutonomousMode.TWO_NOTE_CENTERLINE_FROM_AMP ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose =
              AllianceFlipUtil.apply(TwoNoteCenterlineFromAmpAutoPath.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(TwoNoteCenterlineFromAmpAutoPath(drivetrain, superstructure))
      AutonomousMode.THREE_NOTE_CENTERLINE_FROM_AMP ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose =
              AllianceFlipUtil.apply(TwoNoteCenterlineFromAmpAutoPath.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(ThreeNoteCenterlineFromAmpAutoPath(drivetrain, superstructure))
      AutonomousMode.THREE_NOTE_CENTERLINE_WITHOUT_FIRST_NOTE_FROM_AMP ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose =
              AllianceFlipUtil.apply(
                ThreeNoteCenterlineWithoutFirstNoteAmpAutoPath.startingPose
              )
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(ThreeNoteCenterlineWithoutFirstNoteAmpAutoPath(drivetrain, superstructure))
      AutonomousMode.THREE_NOTE_AND_PICKUP_CENTERLINE_FROM_SOURCE ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose =
              AllianceFlipUtil.apply(ThreeNoteAndPickupCenterlineSourceAutoPath.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(ThreeNoteAndPickupCenterlineSourceAutoPath(drivetrain, superstructure))
      AutonomousMode.THREE_NOTE_AND_PICKUP_CENTERLINE_WITHOUT_FIRST_NOTE_FROM_SOURCE ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose =
              AllianceFlipUtil.apply(
                ThreeNoteAndPickupCenterlineWithoutFirstNoteSourceAutoPath.startingPose
              )
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(
            ThreeNoteAndPickupCenterlineWithoutFirstNoteSourceAutoPath(
              drivetrain, superstructure
            )
          )
      AutonomousMode.PRELOAD_AND_LEAVE_LEFT_SUBWOOFER ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose =
              AllianceFlipUtil.apply(PreloadAndLeaveFromAmpSubwooferAutoPath.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(
            PreloadAndLeaveFromAmpSubwooferAutoPath(
              drivetrain, superstructure, secondaryWaitTime
            )
          )
      AutonomousMode.PRELOAD_AND_LEAVE_RIGHT_SUBWOOFER ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose =
              AllianceFlipUtil.apply(PreloadAndLeaveFromSourceSubwooferAutoPath.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(
            PreloadAndLeaveFromSourceSubwooferAutoPath(
              drivetrain, superstructure, secondaryWaitTime
            )
          )
      AutonomousMode.PRELOAD_AND_LEAVE_CENTER_SUBWOOFER ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            val flippedPose =
              AllianceFlipUtil.apply(PreloadAndLeaveCenterSubwooferAutoPath.startingPose)
            drivetrain.tempZeroGyroYaw(flippedPose.rotation)
            drivetrain.resetFieldFrameEstimator(flippedPose)
          })
          .andThen(
            PreloadAndLeaveCenterSubwooferAutoPath(
              drivetrain, superstructure, secondaryWaitTime
            )
          )
      else -> println("ERROR: unexpected auto mode: $mode")
    }
    return InstantCommand()
  }

  fun getLoadingCommand(drivetrain: Drivetrain): Command {
    return ExamplePathAuto(drivetrain)
  }

  private enum class AutonomousMode {
    TEST_AUTO_PATH,
    FOUR_NOTE_AUTO_PATH_WITH_AMP_SIDE_FIRST,
    FOUR_NOTE_AUTO_PATH_WITH_CENTER_SIDE_FIRST,
    FOUR_NOTE_AUTO_PATH_WITH_SOURCE_SIDE_FIRST,
    FOUR_NOTE_RIGHT_AUTO_PATH,
    FOUR_NOTE_MIDDLE_AUTO_PATH,
    FOUR_NOTE_LEFT_AUTO_PATH,
    TWO_NOTE_CENTERLINE_FROM_SOURCE,
    TWO_NOTE_CENTERLINE_FROM_AMP,
    THREE_NOTE_CENTERLINE_FROM_AMP,
    THREE_NOTE_CENTERLINE_WITHOUT_FIRST_NOTE_FROM_AMP,
    THREE_NOTE_AND_PICKUP_CENTERLINE_FROM_SOURCE,
    THREE_NOTE_AND_PICKUP_CENTERLINE_WITHOUT_FIRST_NOTE_FROM_SOURCE,
    PRELOAD_AND_LEAVE_LEFT_SUBWOOFER,
    PRELOAD_AND_LEAVE_RIGHT_SUBWOOFER,
    PRELOAD_AND_LEAVE_CENTER_SUBWOOFER,
    FIVE_NOTE_AUTO_PATH,
    SIX_NOTE_AUTO_PATH,
    SIX_NOTE_WITH_PICKUP_PATH
  }
}
