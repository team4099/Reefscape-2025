package com.team4099.robot2025

import com.team4099.robot2023.subsystems.led.Leds
import com.team4099.robot2023.subsystems.vision.camera.CameraIO
import com.team4099.robot2023.subsystems.vision.camera.CameraIOPhotonvision
import com.team4099.robot2025.auto.AutonomousSelector
import com.team4099.robot2025.commands.drivetrain.DriveToPose
import com.team4099.robot2025.commands.drivetrain.ReefAimCommand
import com.team4099.robot2025.commands.drivetrain.ReefAlignCommand
import com.team4099.robot2025.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2025.commands.drivetrain.StationAlignCommand
import com.team4099.robot2025.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.FieldConstants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.arm.Arm
import com.team4099.robot2025.subsystems.arm.ArmIOSim
import com.team4099.robot2025.subsystems.arm.ArmIOTalonFX
import com.team4099.robot2025.subsystems.climber.Climber
import com.team4099.robot2025.subsystems.climber.ClimberIO
import com.team4099.robot2025.subsystems.climber.ClimberIOSim
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.drivetrain.drive.DrivetrainIOReal
import com.team4099.robot2025.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2025.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2025.subsystems.drivetrain.gyro.GyroIOPigeon2
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorIOSim
import com.team4099.robot2025.subsystems.elevator.ElevatorIOTalon
import com.team4099.robot2025.subsystems.led.LedIO
import com.team4099.robot2025.subsystems.led.LedIOCandle
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.limelight.LimelightVisionIO
import com.team4099.robot2025.subsystems.rollers.Ramp
import com.team4099.robot2025.subsystems.rollers.RampIOSim
import com.team4099.robot2025.subsystems.rollers.RampIOTalonFX
import com.team4099.robot2025.subsystems.rollers.Rollers
import com.team4099.robot2025.subsystems.rollers.RollersIOSim
import com.team4099.robot2025.subsystems.rollers.RollersIOTalonFX
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.driver.Jessika
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import com.team4099.robot2025.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

object RobotContainer {
  private val drivetrain: Drivetrain
  private val limelight: LimelightVision
  private val arm: Arm
  private val climber: Climber
  private val elevator: Elevator
  private val rollers: Rollers
  private val ramp: Ramp
  private val vision: Vision
  private val leds: Leds
  val superstructure: Superstructure

  init {
    if (RobotBase.isReal()) {
      // Real Hardware Implementations
      // drivetrain = Drivetrain(object: GyroIO {},object: DrivetrainIO {}

      drivetrain = Drivetrain(GyroIOPigeon2, DrivetrainIOReal)
      limelight = LimelightVision(object : LimelightVisionIO {})
      arm = Arm(ArmIOTalonFX)
      climber = Climber(object : ClimberIO {})
      elevator = Elevator(ElevatorIOTalon)
      rollers = Rollers(RollersIOTalonFX)
      ramp = Ramp(RampIOTalonFX)
      leds = Leds(LedIOCandle)

      vision =
        Vision(
          CameraIOPhotonvision(
            VisionConstants.CAMERA_NAMES[0], VisionConstants.CAMERA_TRANSFORMS[0]
          ),
          CameraIOPhotonvision(
            VisionConstants.CAMERA_NAMES[1], VisionConstants.CAMERA_TRANSFORMS[1]
          )
        )
    } else {
      // Simulation implementations
      drivetrain = Drivetrain(object : GyroIO {}, DrivetrainIOSim)
      limelight = LimelightVision(object : LimelightVisionIO {})
      arm = Arm(ArmIOSim)
      climber = Climber(ClimberIOSim)
      elevator = Elevator(ElevatorIOSim)
      rollers = Rollers(RollersIOSim)
      ramp = Ramp(RampIOSim)
      leds = Leds(object : LedIO {})

      vision = Vision(object : CameraIO {})
    }

    vision.setDataInterfaces(
      { drivetrain.fieldTRobot },
      { drivetrain.addVisionData(it) },
      { drivetrain.addSpeakerVisionData(it) }
    )

    superstructure =
      Superstructure(drivetrain, elevator, rollers, ramp, arm, climber, leds, vision, limelight)

    limelight.poseSupplier = { drivetrain.odomTRobot }
  }

  fun mapDefaultCommands() {

    drivetrain.defaultCommand =
      TeleopDriveCommand(
        driver = Jessika(),
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.slowMode },
        drivetrain,
      )

    /*
       ReefAimCommand(
         driver = Jessika(),
         { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
         { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
         { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
         { ControlBoard.slowMode },
         drivetrain,
         superstructure
       )

     */
  }

  fun zeroSteering() {
    drivetrain.zeroSteering()
  }

  fun zeroSensors(isInAutonomous: Boolean = false) {
    drivetrain.currentRequest = DrivetrainRequest.ZeroSensors(isInAutonomous)
  }

  fun zeroAngle(toAngle: Angle) {
    drivetrain.zeroGyroYaw(toAngle)
  }

  fun setSteeringCoastMode() {
    drivetrain.swerveModules.forEach { it.setSteeringBrakeMode(false) }
  }
  fun setSteeringBrakeMode() {
    drivetrain.swerveModules.forEach { it.setSteeringBrakeMode(true) }
  }

  fun setDriveCoastMode() {
    drivetrain.swerveModules.forEach { it.setDriveBrakeMode(false) }
  }

  fun setDriveBrakeMode() {
    drivetrain.swerveModules.forEach { it.setDriveBrakeMode(true) }
  }

  fun requestIdle() {
    superstructure.currentRequest = Request.SuperstructureRequest.Idle()
  }

  fun intakeCoral() {
    superstructure.currentRequest = Request.SuperstructureRequest.IntakeCoral()
  }

  fun requestTuning() {
    superstructure.currentRequest = Request.SuperstructureRequest.Tuning()
  }

  fun mapTeleopControls() {

    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain))
    ControlBoard.intakeCoral.whileTrue(superstructure.intakeCoralCommand())
    ControlBoard.intakeL1.whileTrue(superstructure.intakeL1Command())

    ControlBoard.intakeAlgaeGround.whileTrue(
      superstructure.intakeAlgaeCommand(Constants.Universal.AlgaeLevel.GROUND)
    )
    ControlBoard.intakeAlgaeL3.whileTrue(
      superstructure.intakeAlgaeCommand(Constants.Universal.AlgaeLevel.L3)
    )
    ControlBoard.intakeAlgaeL2.whileTrue(
      superstructure.intakeAlgaeCommand(Constants.Universal.AlgaeLevel.L2)
    )
    // ControlBoard.prepAlgaeBarge.whileTrue(superstructure.prepScoreAlgaeBargeCommand())
    //
    ControlBoard.prepL2.whileTrue(
      superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L2)
    )
    //
    ControlBoard.prepL3.whileTrue(
      superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L3)
    )
    //
    // No L4 scoring in the church
    ControlBoard.prepL4.whileTrue(
      superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4)
    )

    ControlBoard.score.onTrue(superstructure.prepScoreDefaultCommand())
    ControlBoard.score.onFalse(superstructure.scoreCommand())

    ControlBoard.forceIdle.whileTrue(superstructure.requestIdleCommand())

    ControlBoard.eject.whileTrue(superstructure.ejectCommand())

    ControlBoard.alignLeft.whileTrue(
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
        Constants.Universal.BranchSide.LEFT
      )
    )

    ControlBoard.alignRight.whileTrue(
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
        Constants.Universal.BranchSide.RIGHT
      )
    )



    ControlBoard.alignAlgae.whileTrue(
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
        Constants.Universal.BranchSide.CENTER
      )
    )


        ControlBoard.intakeCoral.whileTrue(
          StationAlignCommand(
            driver = Jessika(),
            { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
            { ControlBoard.slowMode },
            drivetrain,
            Constants.Universal.GamePiece.CORAL
          )
        )

        ControlBoard.intakeL1.whileTrue(
          StationAlignCommand(
            driver = Jessika(),
            { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
            { ControlBoard.slowMode },
            drivetrain,
            Constants.Universal.GamePiece.CORAL_L1
          )
        )
  }

  fun mapTestControls() {}

  fun getAutonomousCommand() =
    AutonomousSelector.getCommand(drivetrain, elevator, superstructure, vision)

  fun getAutonomousLoadingCommand() = AutonomousSelector.getLoadingCommand(drivetrain)

  fun resetGyroYawCommand(angle: Angle): Command = ResetGyroYawCommand(drivetrain, angle)

  fun mapTunableCommands() {}
}
