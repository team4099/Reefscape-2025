package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.config.constants.RollersConstants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.FMSData
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds

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
  val branchID: Int
) : Command() {

  lateinit var command: TargetTagCommand
  var scored = false
  var timeScored: Time = -1337.seconds
  var tagID = -1

  init {
    addRequirements(drivetrain)
    addRequirements(vision)
  }

  override fun initialize() {
    tagID = vision.lastTrigVisionUpdate.targetTagID
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
        driver, driveX, driveY, turn, slowMode, drivetrain, vision, horizontalOffset, tagID
      )

    command.initialize()
  }

  override fun execute() {
    command.execute()
    vision.isAligned = false
    vision.isAutoAligning = true

    if (command.isAtSepoint() &&
      superstructure.currentState ==
      Superstructure.Companion.SuperstructureStates.PREP_SCORE_CORAL
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
        .L3_HEIGHT && // just so it doesn't tip when driving and logic stays the same
      DriverStation.isAutonomous()
  }

  override fun end(interrupted: Boolean) {

    vision.isAligned = false
    vision.isAutoAligning = false
    command.end(interrupted)

    CustomLogger.recordDebugOutput("ActiveCommands/TargetReefCommand", false)

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
