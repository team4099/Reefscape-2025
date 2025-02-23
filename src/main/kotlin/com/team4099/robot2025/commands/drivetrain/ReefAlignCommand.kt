package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.FMSData
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerMeter
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterPerSecond
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterPerSecond
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import kotlin.math.hypot

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
        horizontalOffset = if (branchID == 0) {
          VisionConstants.BLUE_REEF_TAG_Y_ALIGNMENTS[tagID]?.first ?: 0.inches
        } else {
          VisionConstants.BLUE_REEF_TAG_Y_ALIGNMENTS[tagID]?.second ?: 0.inches
        }
      } else {
        horizontalOffset = if (branchID == 0) {
          VisionConstants.RED_REEF_TAG_Y_ALIGNMENTS[tagID]?.first ?: 0.inches
        } else {
          VisionConstants.RED_REEF_TAG_Y_ALIGNMENTS[tagID]?.second ?: 0.inches
        }
      }
    }

    command = TargetTagCommand(
      driver,
      driveX,
      driveY,
      turn,
      slowMode,
      drivetrain,
      vision,
      horizontalOffset
    )
  }

  override fun execute() {
    command.execute()

    if (command.isAtSepoint() && superstructure.currentState == Superstructure.Companion.SuperstructureStates.PREP_SCORE_CORAL) {
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
