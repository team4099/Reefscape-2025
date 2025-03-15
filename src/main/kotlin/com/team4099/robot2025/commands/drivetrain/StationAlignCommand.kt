package com.team4099.robot2025.commands.drivetrain

import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.FieldConstants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.FMSData
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.derived.degrees

class StationAlignCommand(
  val driver: DriverProfile,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val slowMode: () -> Boolean,
  val drivetrain: Drivetrain,
  val desiredGamePiece: Constants.Universal.GamePiece
) : Command() {

  lateinit var command: TargetAngleCommand

  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {

    val targetAngle =
      if (FMSData.isBlue) VisionConstants.BLUE_STATION_ALIGN_THETA
      else VisionConstants.RED_STATION_ALIGN_THETA

    val stationSideFlip = if (drivetrain.fieldTRobot.y > FieldConstants.fieldWidth / 2) -1 else 1

    command =
      TargetAngleCommand(
        driver,
        driveX,
        driveY,
        turn,
        slowMode,
        drivetrain,
        { targetAngle * stationSideFlip }
      )
  }

  override fun execute() {
    command.execute()
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {

    command.end(interrupted)

    CustomLogger.recordDebugOutput("ActiveCommands/StationAlignCommand", false)
    drivetrain.currentRequest =
      Request.DrivetrainRequest.OpenLoop(
        driver.rotationSpeedClampedSupplier(turn, slowMode),
        driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
        fieldOriented = true
      )
  }
}
