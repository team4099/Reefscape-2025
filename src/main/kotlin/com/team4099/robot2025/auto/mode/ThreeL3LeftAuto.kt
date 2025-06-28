package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.drivetrain.DrivePathCommand
import com.team4099.robot2025.commands.drivetrain.ReefAlignCommand
import com.team4099.robot2025.commands.drivetrain.ResetPoseCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.TrajectoryTypes
import com.team4099.robot2025.util.driver.Jessika
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.smoothDeadband

class ThreeL3LeftAuto(
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
      )
        .withTimeout(
          1.1
        ), // timeout @ total time so we're not wasting time correcting when auto align
      // will correct
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
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L3)
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain, TrajectoryTypes.Choreo(secondTrajectory), keepTrapping = false
        ),
        WaitCommand(1.0).andThen(superstructure.intakeCoralCommand())
      ),
      WaitCommand(1.0).until {
        superstructure.theoreticalGamePiece == Constants.Universal.GamePiece.CORAL
      },
      DrivePathCommand.createPathInFieldFrame(
        drivetrain, TrajectoryTypes.Choreo(thirdTrajectory), keepTrapping = false
      )
        .withTimeout(1.1),
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
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L3)
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain, TrajectoryTypes.Choreo(fourthTrajectory), keepTrapping = false
        ),
        WaitCommand(1.0).andThen(superstructure.intakeCoralCommand())
      ),
      WaitCommand(1.0).until {
        superstructure.theoreticalGamePiece == Constants.Universal.GamePiece.CORAL
      },
      DrivePathCommand.createPathInFieldFrame(
        drivetrain, TrajectoryTypes.Choreo(fifthTrajectory), keepTrapping = false
      )
        .withTimeout(1.1),
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
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L3)
      )
    )
  }
  companion object {
    private val firstTrajectory =
      Choreo.loadTrajectory<SwerveSample>("ThreeL4Home/startingLineTo1Left").get()
    private val secondTrajectory = Choreo.loadTrajectory<SwerveSample>("ThreeL4Home/1to2Left").get()
    private val thirdTrajectory = Choreo.loadTrajectory<SwerveSample>("ThreeL4Home/2to3Left").get()
    private val fourthTrajectory = Choreo.loadTrajectory<SwerveSample>("ThreeL4Home/3to4Left").get()
    private val fifthTrajectory = Choreo.loadTrajectory<SwerveSample>("ThreeL4Home/4to5Left").get()
  }
}
