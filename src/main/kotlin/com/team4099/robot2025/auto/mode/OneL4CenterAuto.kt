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
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class OneL4CenterAuto(
    val drivetrain: Drivetrain,
    val elevator: Elevator,
    val superstructure: Superstructure,
    val vision: Vision
) : SequentialCommandGroup() {
    init {
        addRequirements(drivetrain)

        addCommands(
            WaitCommand(3.0),
            DrivePathCommand.createPathInFieldFrame(
                drivetrain, TrajectoryTypes.Choreo(firstTrajectory), keepTrapping = false
            )
                .withTimeout(
                    1.5
                ), // timeout @ total time so we're not wasting time correcting when auto align
            // will correct
            WaitCommand(1.0),
//            ReefAlignCommand(
//                driver = Jessika(),
//                { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
//                { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
//                { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
//                { ControlBoard.slowMode },
//                drivetrain,
//                elevator,
//                superstructure,
//                vision,
//                1
//            ).withTimeout(3.0),
//            superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4),
//            superstructure.scoreCommand()
        )
    }
    companion object {
        private val firstTrajectory =
            Choreo.loadTrajectory<SwerveSample>("ThreeL4Home/center1").get()
        val startingPose = Pose2d(
            Translation2d(
                7.157601833343506.meters,
                3.9998908042907715.meters
            ),
            180.degrees
        )
    }
}
