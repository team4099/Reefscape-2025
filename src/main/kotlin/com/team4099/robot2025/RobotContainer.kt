package com.team4099.robot2023

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2023.commands.drivetrain.TargetAngleCommand
import com.team4099.robot2023.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2023.config.ControlBoard
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOReal
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIOPigeon2
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.elevator.ElevatorIONEO
import com.team4099.robot2023.subsystems.elevator.ElevatorIOSim
import com.team4099.robot2023.subsystems.feeder.Feeder
import com.team4099.robot2023.subsystems.feeder.FeederIONeo
import com.team4099.robot2023.subsystems.feeder.FeederIOSim
import com.team4099.robot2023.subsystems.flywheel.Flywheel
import com.team4099.robot2023.subsystems.flywheel.FlywheelIOSim
import com.team4099.robot2023.subsystems.flywheel.FlywheelIOTalon
import com.team4099.robot2023.subsystems.intake.Intake
import com.team4099.robot2023.subsystems.intake.IntakeIOFalconNEO
import com.team4099.robot2023.subsystems.intake.IntakeIOSim
import com.team4099.robot2023.subsystems.limelight.LimelightVision
import com.team4099.robot2023.subsystems.limelight.LimelightVisionIO
import com.team4099.robot2023.subsystems.limelight.LimelightVisionIOReal
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.subsystems.vision.Vision
import com.team4099.robot2023.subsystems.vision.camera.CameraIO
import com.team4099.robot2023.subsystems.wrist.Wrist
import com.team4099.robot2023.subsystems.wrist.WristIOSim
import com.team4099.robot2023.subsystems.wrist.WristIOTalon
import com.team4099.robot2023.util.driver.Jessika
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import com.team4099.robot2023.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

object RobotContainer {
  private val drivetrain: Drivetrain
  private val vision: Vision
  private val limelight: LimelightVision
  private val intake: Intake
  private val feeder: Feeder
  private val elevator: Elevator
  private val flywheel: Flywheel
  private val wrist: Wrist
  val superstructure: Superstructure

  val rumbleState
    get() = feeder.rumbleTrigger
  var setClimbAngle = -1337.degrees
  var setAmpAngle = 270.0.degrees
  var climbAngle: () -> Angle = { setClimbAngle }
  var ampAngle: () -> Angle = { setAmpAngle }

  val podiumAngle =
    LoggedTunableValue(
      "Defense/PodiumShotAngle", 25.0.degrees, Pair({ it.inDegrees }, { it.degrees })
    )

  init {
    if (RobotBase.isReal()) {
      // Real Hardware Implementations
      // drivetrain = Drivetrain(object: GyroIO {},object: DrivetrainIO {}

      drivetrain = Drivetrain(GyroIOPigeon2, DrivetrainIOReal)
      vision = Vision(object : CameraIO {})
      limelight = LimelightVision(LimelightVisionIOReal)
      intake = Intake(IntakeIOFalconNEO)
      feeder = Feeder(FeederIONeo)
      elevator = Elevator(ElevatorIONEO)
      flywheel = Flywheel(FlywheelIOTalon)
      wrist = Wrist(WristIOTalon)
    } else {
      // Simulation implementations
      drivetrain = Drivetrain(object : GyroIO {}, DrivetrainIOSim)
      vision = Vision(object : CameraIO {})
      limelight = LimelightVision(object : LimelightVisionIO {})
      intake = Intake(IntakeIOSim)
      feeder = Feeder(FeederIOSim)
      elevator = Elevator(ElevatorIOSim)
      flywheel = Flywheel(FlywheelIOSim)
      wrist = Wrist(WristIOSim)
    }

    superstructure =
      Superstructure(intake, feeder, elevator, wrist, flywheel, drivetrain, vision, limelight)
    vision.setDataInterfaces(
      { drivetrain.fieldTRobot },
      { drivetrain.addVisionData(it) },
      { drivetrain.addSpeakerVisionData(it) }
    )
    vision.drivetrainOdometry = { drivetrain.odomTRobot }
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
        drivetrain
      )
    /*
    module steeing tuning

    drivetrain.defaultCommand =
      SwerveModuleTuningCommand(
        drivetrain,
        { (ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) * 180).degrees },
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

  fun mapTeleopControls() {
    limelight.limelightState = LimelightVision.LimelightStates.TARGETING_GAME_PIECE

    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain))

    ControlBoard.intake.whileTrue(superstructure.groundIntakeCommand())

    /*
    ControlBoard.intake.whileTrue(
      ParallelCommandGroup(
        superstructure.groundIntakeCommand(),
        TargetNoteCommand(
          driver = Jessika(),
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { ControlBoard.slowMode },
          drivetrain,
          limelight,
          feeder
        )
      )
    )

     */

    ControlBoard.prepAmp.whileTrue(superstructure.prepAmpCommand())

    ControlBoard.passingShotAlignment.whileTrue(
      TargetAngleCommand(
        driver = Jessika(),
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.slowMode },
        drivetrain,
        {
          if (DriverStation.getAlliance().isPresent &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red
          )
            225.degrees + 180.degrees
          else if (DriverStation.getAlliance().isPresent &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
          )
            135.degrees
          else 135.degrees
        },
      )
    )

    ControlBoard.prepHighProtected.whileTrue(
      ParallelCommandGroup(
        superstructure.prepSpeakerMidCommand(),
        TargetAngleCommand(
          driver = Jessika(),
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { ControlBoard.slowMode },
          drivetrain,
          {
            if (DriverStation.getAlliance().isPresent &&
              DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            )
              podiumAngle.get()
            else 180.degrees - podiumAngle.get()
          },
        )
      )
    )
    ControlBoard.prepHigh.whileTrue(superstructure.prepSpeakerHighCommand())
    ControlBoard.score.whileTrue(superstructure.scoreCommand())
    ControlBoard.extendClimb.whileTrue(superstructure.climbExtendCommand())
    ControlBoard.retractClimb.whileTrue(superstructure.climbRetractCommand())
    ControlBoard.forceIdle.whileTrue(superstructure.requestIdleCommand())
    ControlBoard.prepLow.whileTrue(superstructure.prepSpeakerLowCommand())
    ControlBoard.ejectGamePiece.whileTrue(superstructure.ejectGamePieceCommand())
    ControlBoard.passingShot.whileTrue(superstructure.passingShotCommand())
    ControlBoard.underStagePassingShot.whileTrue(superstructure.underStageCommand())

    ControlBoard.targetAmp.whileTrue(
      TargetAngleCommand(
        driver = Jessika(),
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.slowMode },
        drivetrain,
        ampAngle
      )
    )

    ControlBoard.climbAlignFar.whileTrue(
      runOnce({
        setClimbAngle =
          if (DriverStation.getAlliance().isPresent &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red
          )
            180.degrees
          else (0).degrees
      })
    )

    ControlBoard.climbAlignLeft.whileTrue(
      runOnce({
        setClimbAngle =
          if (DriverStation.getAlliance().isPresent &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red
          )
            300.degrees
          else (120).degrees
      })
    )

    ControlBoard.climbAlignRight.whileTrue(
      runOnce({
        setClimbAngle =
          if (DriverStation.getAlliance().isPresent &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red
          )
            (60).degrees
          else 240.degrees
      })
    )
    ControlBoard.climbAutoAlign.whileTrue(
      TargetAngleCommand(
        driver = Jessika(),
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.slowMode },
        drivetrain,
        climbAngle
      )
    )
    //    ControlBoard.climbAlignLeft.whileTrue(
    //      TargetAngleCommand(
    //        driver = Jessika(),
    //        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
    //        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
    //        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
    //        { ControlBoard.slowMode },
    //        drivetrain,
    //        if (DriverStation.getAlliance().isPresent &&
    //          DriverStation.getAlliance().get() == DriverStation.Alliance.Red
    //        )
    //          120.degrees
    //        else -60.degrees
    //      )
    //    )
    //
    //    ControlBoard.climbAlignRight.whileTrue(
    //      TargetAngleCommand(
    //        driver = Jessika(),
    //        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
    //        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
    //        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
    //        { ControlBoard.slowMode },
    //        drivetrain,
    //        if (DriverStation.getAlliance().isPresent &&
    //          DriverStation.getAlliance().get() == DriverStation.Alliance.Red
    //        )
    //          -120.0.degrees
    //        else 60.0.degrees,
    //      )
    //    )

    /*

    ControlBoard.targetSpeaker.whileTrue(
      ParallelCommandGroup(
        TargetSpeakerCommand(
          driver = Jessika(),
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { ControlBoard.slowMode },
          drivetrain,
          superstructure,
          vision
        ),
        superstructure.autoAimCommand()
      )
    )

     */

    // ControlBoard.lockWheels.whileTrue(LockDriveCommand(drivetrain))

    //    ControlBoard.characterizeSubsystem.whileTrue(
    //      WheelRadiusCharacterizationCommand(
    //        drivetrain, WheelRadiusCharacterizationCommand.Companion.Direction.CLOCKWISE
    //      )
    //    )

    /*
    TUNING COMMANDS
    ControlBoard.testIntake.whileTrue(superstructure.testIntakeCommand())
    ControlBoard.testFeederIntake.whileTrue(superstructure.testFeederIntakeCommand())
    ControlBoard.testFeederShoot.whileTrue(superstructure.testFeederShootCommand())
    ControlBoard.testFlywheel.whileTrue(superstructure.testFlywheelCommand())
    ControlBoard.testWrist.whileTrue(superstructure.testWristCommand())
    ControlBoard.testElevator.whileTrue(superstructure.testElevatorCommand())
    */

    /*

    <<<<<<< HEAD
        ControlBoard.shooterDown.whileTrue(flywheel.flywheelSpinUpCommand())
        ControlBoard.shooterUp.whileTrue(flywheel.flywheelStopCommand())
        ControlBoard.wristTestUp.whileTrue(wrist.wristPositionUpCommand())
        ControlBoard.wristTestDown.whileTrue(wrist.wristPositionDownCommand())
        ControlBoard.feederTest.whileTrue(feeder.feederOpenLoopShootTestCommand())
        ControlBoard.elevatorDown.whileTrue(elevator.elevatorClosedLoopRetractCommand())
        ControlBoard.elevatorUp.whileTrue(elevator.testElevatorClosedLoopExtendCommand())
        ControlBoard.setTuningMode.whileTrue(superstructure.tuningCommand())
         */
  }

  fun mapTestControls() {}

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain, superstructure)

  fun getAutonomousLoadingCommand() = AutonomousSelector.getLoadingCommand(drivetrain)

  fun resetGyroYawCommand(angle: Angle): Command = ResetGyroYawCommand(drivetrain, angle)

  fun mapTunableCommands() {}
}
