package com.team4099.robot2025.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2025.commands.drivetrain.DrivePathCommand
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.util.CustomTrajectory
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class ExamplePathAuto(
  val drivetrain: Drivetrain
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand.createPathInFieldFrame(
        drivetrain,
        CustomTrajectory.fromWaypoints(
          drivetrain,
          {
            listOf(
              FieldWaypoint(Translation2d(0.0.meters, 0.0.meters).translation2d, 0.0.degrees.inRotation2ds, 0.0.degrees.inRotation2ds),
              FieldWaypoint(Translation2d(1.0.meters, 1.0.meters).translation2d, 0.0.degrees.inRotation2ds, 0.0.degrees.inRotation2ds)
            )
          }
        )
      )
    )
  }
}
