package com.team4099.robot2025.commands.drivetrain

import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.FieldConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.FMSData
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.radians
import kotlin.math.atan2

class ReefAimCommand(
  val driver: DriverProfile,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val slowMode: () -> Boolean,
  val drivetrain: Drivetrain,
  val superstructure: Superstructure
) : Command() {

  var aimCommand: TargetAngleCommand = TargetAngleCommand(driver, driveX, driveY, turn, slowMode, drivetrain, { getTargetAngle() })
  var defaultCommand: TeleopDriveCommand = TeleopDriveCommand(driver, driveX, driveY, turn, slowMode, drivetrain)

  init {
    addRequirements(drivetrain)
  }

  fun getTargetAngle(): Angle {
    val targetPose =
      if (FMSData.isBlue) FieldConstants.REEF.blue_center else FieldConstants.REEF.red_center
    Logger.recordOutput("Vision/targetpose", targetPose.translation2d)
    val robotPose = drivetrain.fieldTRobot.translation
    val x = (targetPose - robotPose).x
    val y = (targetPose - robotPose).y
    return atan2(y.inMeters, x.inMeters).radians
  }

  override fun initialize() {}

  override fun execute() {
    if (true
    ) {
      aimCommand.execute()
    } else {
      defaultCommand.execute()
    }
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {

    aimCommand.end(interrupted)
    defaultCommand.end(interrupted)

    CustomLogger.recordDebugOutput("ActiveCommands/ReefAimCommand", false)
    drivetrain.currentRequest =
      Request.DrivetrainRequest.OpenLoop(
        driver.rotationSpeedClampedSupplier(turn, slowMode),
        driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
        fieldOriented = true
      )
  }
}
