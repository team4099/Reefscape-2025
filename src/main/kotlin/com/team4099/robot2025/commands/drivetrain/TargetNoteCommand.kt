package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.feeder.Feeder
import com.team4099.robot2023.subsystems.limelight.LimelightVision
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.util.CustomLogger
import com.team4099.robot2023.util.driver.DriverProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import kotlin.math.hypot

class TargetNoteCommand(
  val driver: DriverProfile,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val slowMode: () -> Boolean,
  val drivetrain: Drivetrain,
  val limelight: LimelightVision,
  val feeder: Feeder
) : Command() {

  private var thetaPID: PIDController<Radian, Velocity<Radian>>
  val thetakP =
    LoggedTunableValue(
      "NoteAlignment/noteThetakP",
      Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
    )
  val thetakI =
    LoggedTunableValue(
      "NoteAlignment/noteThetakI",
      Pair(
        { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
      )
    )
  val thetakD =
    LoggedTunableValue(
      "NoteAlignment/noteThetakD",
      Pair(
        { it.inDegreesPerSecondPerDegreePerSecond },
        { it.degrees.perSecond.perDegreePerSecond }
      )
    )

  init {
    addRequirements(drivetrain)

    thetaPID =
      PIDController(
        thetakP.get(),
        thetakI.get(),
        thetakD.get(),
      )

    if (!(RobotBase.isSimulation())) {

      thetakP.initDefault(DrivetrainConstants.PID.LIMELIGHT_THETA_KP)
      thetakI.initDefault(DrivetrainConstants.PID.LIMELIGHT_THETA_KI)
      thetakD.initDefault(DrivetrainConstants.PID.LIMELIGHT_THETA_KD)

      thetaPID =
        PIDController(
          DrivetrainConstants.PID.LIMELIGHT_THETA_KP,
          DrivetrainConstants.PID.LIMELIGHT_THETA_KI,
          DrivetrainConstants.PID.LIMELIGHT_THETA_KD
        )
    } else {
      thetakP.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)

      thetaPID =
        PIDController(
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD
        )
    }

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    thetaPID.reset()

    /*
    if (thetakP.hasChanged() || thetakI.hasChanged() || thetakD.hasChanged()) {
      thetaPID = PIDController(thetakP.get(), thetakI.get(), thetakD.get())
    }

     */
  }

  override fun execute() {

    drivetrain.defaultCommand.end(true)

    Logger.recordOutput("ActiveCommands/TargetNoteCommand", true)

    val thetaFeedback = thetaPID.calculate(limelight.targetGamePieceTx ?: 0.0.degrees, 0.0.degrees)
    CustomLogger.recordDebugOutput("NoteAlignment/error", thetaPID.error.inDegrees)
    CustomLogger.recordDebugOutput("NoteAlignment/thetaFeedback", thetaFeedback.inDegreesPerSecond)

    if (!feeder.hasNote && limelight.inputs.gamePieceTargets.size > 0) {
      val driveVector = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
      var autoDriveVector =
        hypot(driveVector.first.inMetersPerSecond, driveVector.second.inMetersPerSecond)
      if (DriverStation.getAlliance().isPresent &&
        DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
      ) {
        autoDriveVector =
          -hypot(driveVector.first.inMetersPerSecond, driveVector.second.inMetersPerSecond)
      }
      drivetrain.currentRequest =
        Request.DrivetrainRequest.OpenLoop(
          thetaFeedback,
          Pair(autoDriveVector.meters.perSecond, 0.0.meters.perSecond),
          fieldOriented = false
        )
    } else {
      val speed = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
      val rotation = driver.rotationSpeedClampedSupplier(turn, slowMode)

      drivetrain.currentRequest = Request.DrivetrainRequest.OpenLoop(rotation, speed)
    }
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    Logger.recordOutput("ActiveCommands/TargetAngleCommand", false)
    drivetrain.currentRequest =
      Request.DrivetrainRequest.OpenLoop(
        driver.rotationSpeedClampedSupplier(turn, slowMode),
        driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
        fieldOriented = true
      )
  }
}
