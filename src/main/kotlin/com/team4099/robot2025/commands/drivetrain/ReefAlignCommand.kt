package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.config.constants.FieldConstants
import com.team4099.robot2025.config.constants.RollersConstants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.FMSData
import com.team4099.robot2025.util.Velocity2d
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import org.eclipse.jetty.util.log.Log
import org.littletonrobotics.junction.Logger
import org.team4099.lib.apriltag.AprilTag
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees

class ReefAlignCommand(
  val driver: DriverProfile,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val slowMode: () -> Boolean,
  val drivetrain: Drivetrain,
  val elevator: Elevator,
  val superstructure: Superstructure,
  val vision: Vision,
  val branchSide: Constants.Universal.BranchSide,
  val forceTagId: Int = -1
) : Command() {

  lateinit var driveCommand: DriveToPose

  var timeScored: Time = -1337.seconds
  var scored = false
  var closestTag = Pair(-1, Pose2d())
  var goalPose = Pose2d()

  init {
    addRequirements(drivetrain)
    addRequirements(vision)
  }

  fun getClosestTag(robotPose: Pose2d, robotVelocity: Velocity2d): Pair<Int, Pose2d>  {

    val predictedRobotPose = robotPose.transformBy(
      Transform2d(
        Translation2d(robotVelocity.velocity2dWPIlib * DrivetrainConstants.REEF_VELOCITY_PREDICTION_TIME.inSeconds),
        0.0.degrees
      )
    )

    val reefTagIDs = if(FMSData.isBlue) VisionConstants.BLUE_REEF_TAGS else VisionConstants.RED_REEF_TAGS

    val reefTagPoses = mutableMapOf<Int, Pose2d>()
    for (id in reefTagIDs) {
      reefTagPoses.put(id, FieldConstants.AprilTagLayoutType.OFFICIAL.layout.getTagPose(id).toPose2d())
    }

    return reefTagPoses.minByOrNull {(predictedRobotPose.translation - it.value.translation).magnitude }?.toPair()
      ?: Pair(-1, robotPose)
  }

  fun getRobotPose(): Pose2d {
    val trigVisionUpdate = vision.lastTrigVisionUpdate

    if (trigVisionUpdate.targetTagID != -1
      && trigVisionUpdate.targetTagID == closestTag.first
      && Clock.fpgaTime - trigVisionUpdate.timestamp < 0.2.seconds) {

      Logger.recordOutput("ReefAlign/PoseFeedbackType", "SingleTag")
      return trigVisionUpdate.fieldTRobot
    } else {

      Logger.recordOutput("ReefAlign/PoseFeedbackType", "FieldFrameEstimation")
      return drivetrain.fieldTRobot
    }
  }

  fun getDriverRelativeBranchOffset(tagID: Int): Length {
    var offset = 0.0.inches
    if (FMSData.isBlue) {
        if (branchSide == Constants.Universal.BranchSide.LEFT) {
          offset = VisionConstants.BLUE_REEF_TAG_Y_ALIGNMENTS[tagID]?.first ?: 0.inches
        } else if(branchSide == Constants.Universal.BranchSide.RIGHT) {
          offset = VisionConstants.BLUE_REEF_TAG_Y_ALIGNMENTS[tagID]?.second ?: 0.inches
        }
    } else {
        if (branchSide == Constants.Universal.BranchSide.LEFT) {
          offset = VisionConstants.RED_REEF_TAG_Y_ALIGNMENTS[tagID]?.first ?: 0.inches
        } else if (branchSide == Constants.Universal.BranchSide.RIGHT) {
          offset = VisionConstants.RED_REEF_TAG_Y_ALIGNMENTS[tagID]?.second ?: 0.inches
        }
    }

    return offset
  }

  override fun initialize() {
    closestTag = if (forceTagId == -1) getClosestTag(drivetrain.fieldTRobot, drivetrain.fieldVelocity) else Pair(forceTagId, FieldConstants.AprilTagLayoutType.OFFICIAL.layout.getTagPose(forceTagId).toPose2d())

    if (closestTag.first != -1) {
      vision.currentRequest = Request.VisionRequest.TargetTag(arrayOf(closestTag.first))
    }

    val xOffset = DrivetrainConstants.DRIVETRAIN_WIDTH / 2 + 3.inches
    val yOffset = getDriverRelativeBranchOffset(closestTag.first)


    goalPose = closestTag.second

      .transformBy(
      Transform2d(
        Translation2d(xOffset, yOffset), 180.degrees
      )
    )



    driveCommand = DriveToPose(
      drivetrain,
      {goalPose},
      {getRobotPose()}
    )

    driveCommand.initialize()

    Logger.recordOutput("ReefAlignment/targetTagID", closestTag.first)
    Logger.recordOutput("ReefAlignment/goalPose", goalPose.pose2d)
  }

  override fun execute() {
    driveCommand.execute()
    vision.isAligned = false
    vision.isAutoAligning = true

    if (driveCommand.atTarget() &&
      driveCommand.withinTolerance(DrivetrainConstants.REEF_POSITION_TOLERANCE, DrivetrainConstants.REEF_THETA_TOLERANCE) &&
      superstructure.currentState == Superstructure.Companion.SuperstructureStates.PREP_SCORE_CORAL
    ) {
      vision.isAligned = true
      superstructure.currentRequest = Request.SuperstructureRequest.Score()

      if (DriverStation.isAutonomous()) {
        scored = true

        if (timeScored == -1337.seconds) {
          timeScored = Clock.fpgaTime
        }
      }
    }
  }

  override fun isFinished(): Boolean {
    return scored &&
      Clock.fpgaTime - timeScored > RollersConstants.CORAL_SPIT_TIME &&
      elevator.inputs.elevatorPosition <=
      ElevatorConstants
        .PREP_L4_HEIGHT && // just so it doesn't tip when driving and logic stays the same
      DriverStation.isAutonomous()
  }

  override fun end(interrupted: Boolean) {

    vision.currentRequest = Request.VisionRequest.TargetReef()

    vision.isAligned = false
    vision.isAutoAligning = false
    driveCommand.end(interrupted)

    CustomLogger.recordDebugOutput("ActiveCommands/ReefAlignCommand", false)

    if (!DriverStation.isAutonomous()) {
      drivetrain.currentRequest =
        Request.DrivetrainRequest.OpenLoop(
          driver.rotationSpeedClampedSupplier(turn, slowMode),
          driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
          fieldOriented = true
        )
    }
  }
}
