package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.drivetrain.DrivePathCommand
import com.team4099.robot2025.commands.drivetrain.ReefAlignCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.TrajectoryTypes
import com.team4099.robot2025.util.driver.Jessika
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class ThreeL4LeftAuto(
  val drivetrain: Drivetrain,
  val elevator: Elevator,
  val superstructure: Superstructure,
  val vision: Vision
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand.createPathInFieldFrame(
        drivetrain, TrajectoryTypes.Choreo(firstTrajectory), keepTrapping = false
      ).withTimeout(1.6),
      ParallelCommandGroup(
        ReefAlignCommand(
          driver = Jessika(),
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { ControlBoard.slowMode },
          drivetrain,
          elevator,
          superstructure,
          vision,
          1
        ),
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4)
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain, TrajectoryTypes.Choreo(secondTrajectory), keepTrapping = false
        ).withTimeout(4.2),
        WaitCommand(1.6).andThen(superstructure.intakeCoralCommand())
      ),
      ParallelCommandGroup(
        ReefAlignCommand(
          driver = Jessika(),
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { ControlBoard.slowMode },
          drivetrain,
          elevator,
          superstructure,
          vision,
          1
        ),
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4)
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain, TrajectoryTypes.Choreo(thirdTrajectory), keepTrapping = false
        ).withTimeout(4.0),
        WaitCommand(1.6).andThen(superstructure.intakeCoralCommand())
      ),
      ParallelCommandGroup(
        ReefAlignCommand(
          driver = Jessika(),
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { ControlBoard.slowMode },
          drivetrain,
          elevator,
          superstructure,
          vision,
          0
        ),
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4)
      )
    )
  }
  companion object {
    private val firstTrajectory =
      Choreo.loadTrajectory<SwerveSample>("ThreeL4Home/startingLineTo1Left").get()
    private val secondTrajectory = Choreo.loadTrajectory<SwerveSample>("ThreeL4Home/1to2Left").get()
    private val thirdTrajectory = Choreo.loadTrajectory<SwerveSample>("ThreeL4Home/2to3Left").get()

    val startingPose = Pose2d(Translation2d(firstTrajectory.getInitialPose(false).get().translation), 180.degrees)
    val secondPose = Pose2d(secondTrajectory.getInitialPose(false).get())
    val thirdPose = Pose2d(thirdTrajectory.getInitialPose(false).get())
  }
}
