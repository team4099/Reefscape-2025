package com.team4099.robot2025.commands.drivetrain

import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.FMSData
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.base.inches

class ReefAlignCommand(
  val driver: DriverProfile,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val slowMode: () -> Boolean,
  val drivetrain: Drivetrain,
  val superstructure: Superstructure,
  val vision: Vision,
  val branchID: Int
) : Command() {

  lateinit var command: TargetTagCommand

  init {
    addRequirements(drivetrain)
    addRequirements(vision)
  }

  override fun initialize() {
    val tagID = vision.lastTrigVisionUpdate.targetTagID
    var horizontalOffset = 0.inches
    if (DriverStation.getAlliance().isPresent) {
      if (FMSData.isBlue) {
        horizontalOffset =
          if (branchID == 0) {
            VisionConstants.BLUE_REEF_TAG_Y_ALIGNMENTS[tagID]?.first ?: 0.inches
          } else {
            VisionConstants.BLUE_REEF_TAG_Y_ALIGNMENTS[tagID]?.second ?: 0.inches
          }
      } else {
        horizontalOffset =
          if (branchID == 0) {
            VisionConstants.RED_REEF_TAG_Y_ALIGNMENTS[tagID]?.first ?: 0.inches
          } else {
            VisionConstants.RED_REEF_TAG_Y_ALIGNMENTS[tagID]?.second ?: 0.inches
          }
      }
    }

    command =
      TargetTagCommand(
        driver, driveX, driveY, turn, slowMode, drivetrain, vision, horizontalOffset
      )
  }

  override fun execute() {
    command.execute()

    if (command.isAtSepoint() &&
      superstructure.currentState ==
      Superstructure.Companion.SuperstructureStates.PREP_SCORE_CORAL
    ) {
      superstructure.currentRequest = Request.SuperstructureRequest.Score()
    }
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {

    command.end(interrupted)

    CustomLogger.recordDebugOutput("ActiveCommands/TargetReefCommand", false)
    drivetrain.currentRequest =
      Request.DrivetrainRequest.OpenLoop(
        driver.rotationSpeedClampedSupplier(turn, slowMode),
        driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
        fieldOriented = true
      )
  }
}
