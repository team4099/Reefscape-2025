package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.drivetrain.DrivePathCommand
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.util.TrajectoryTypes
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class ExamplePathAuto(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand.createPathInFieldFrame(
        drivetrain,
        TrajectoryTypes.Choreo(
          Choreo.loadTrajectory<SwerveSample>("ThreeL4Home/startingLineTo1.traj").get()
        )
      )
    )
  }
}
