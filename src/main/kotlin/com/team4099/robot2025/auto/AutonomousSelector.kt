package com.team4099.robot2025.auto

import com.team4099.robot2025.auto.mode.ExamplePathAuto
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
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

    autonomousModeChooser.addOption("Example Path Auto", AutonomousMode.EXAMPLE_PATH_AUTO)

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

  fun getCommand(drivetrain: Drivetrain): Command {
    val mode = autonomousModeChooser.get()

    when (mode) {
      // Delete this when real autos are made
      AutonomousMode.EXAMPLE_PATH_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.tempZeroGyroYaw(ExamplePathAuto.startingPose.rotation)
            drivetrain.resetFieldFrameEstimator(
              AllianceFlipUtil.apply(ExamplePathAuto.startingPose)
            )
          })
      else -> println("ERROR: unexpected auto mode: $mode")
    }
    return InstantCommand()
  }

  fun getLoadingCommand(drivetrain: Drivetrain): Command {
    return ExamplePathAuto(drivetrain)
  }

  private enum class AutonomousMode {
    // Delete this when real autos are made
    EXAMPLE_PATH_AUTO
  }
}
