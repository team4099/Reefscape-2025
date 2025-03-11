package com.team4099.robot2025.auto

import com.team4099.robot2025.auto.mode.ExamplePathAuto
import com.team4099.robot2025.auto.mode.ThreeL4HomeAuto
import com.team4099.robot2025.auto.mode.ThreeL4LeftAuto
import com.team4099.robot2025.auto.mode.ThreeL4RightAuto
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.AllianceFlipUtil
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
      "Three L4 Auto from Processor Side (Default)", AutonomousMode.THREE_L4_LEFT_AUTO
    )

    autonomousModeChooser.addOption(
      "Three L4 Auto from Far Side", AutonomousMode.THREE_L4_RIGHT_AUTO
    )

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

  fun getCommand(
    drivetrain: Drivetrain,
    elevator: Elevator,
    superstructure: Superstructure,
    vision: Vision
  ): Command {
    val mode = autonomousModeChooser.get()

    when (mode) {
      // Delete this when real autos are made
      AutonomousMode.THREE_L4_HOME_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.tempZeroGyroYaw(
              AllianceFlipUtil.apply(ThreeL4HomeAuto.startingPose).rotation
            )
            drivetrain.resetFieldFrameEstimator(
              AllianceFlipUtil.apply(ThreeL4HomeAuto.startingPose)
            )
          })
          .andThen(ThreeL4HomeAuto(drivetrain, elevator, superstructure, vision))
      AutonomousMode.THREE_L4_LEFT_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.tempZeroGyroYaw(
              AllianceFlipUtil.apply(ThreeL4LeftAuto.startingPose).rotation
            )
            drivetrain.resetFieldFrameEstimator(
              AllianceFlipUtil.apply(ThreeL4LeftAuto.startingPose)
            )
          })
          .andThen(ThreeL4LeftAuto(drivetrain, elevator, superstructure, vision))
      AutonomousMode.THREE_L4_RIGHT_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.tempZeroGyroYaw(
              AllianceFlipUtil.apply(ThreeL4RightAuto.startingPose).rotation
            )
            drivetrain.resetFieldFrameEstimator(
              AllianceFlipUtil.apply(ThreeL4RightAuto.startingPose)
            )
          })
          .andThen(ThreeL4RightAuto(drivetrain, elevator, superstructure, vision))
      else -> return InstantCommand()
    }
  }

  fun getLoadingCommand(drivetrain: Drivetrain): Command {
    return ExamplePathAuto(drivetrain)
  }

  private enum class AutonomousMode {
    // Delete this when real autos are made
    THREE_L4_HOME_AUTO,
    THREE_L4_LEFT_AUTO,
    THREE_L4_RIGHT_AUTO
  }
}
