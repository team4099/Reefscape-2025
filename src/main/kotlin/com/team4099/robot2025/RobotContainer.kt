package com.team4099.robot2025

import com.team4099.robot2023.subsystems.limelight.LimelightVisionIOReal
import com.team4099.robot2025.auto.AutonomousSelector
import com.team4099.robot2025.commands.characterization.TestElevatorCommand
import com.team4099.robot2025.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2025.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.arm.Arm
import com.team4099.robot2025.subsystems.arm.ArmIO
import com.team4099.robot2025.subsystems.arm.ArmIOSim
import com.team4099.robot2025.subsystems.arm.ArmIOTalonFX
import com.team4099.robot2025.subsystems.climber.Climber
import com.team4099.robot2025.subsystems.climber.ClimberIO
import com.team4099.robot2025.subsystems.climber.ClimberIOSim
import com.team4099.robot2025.subsystems.climber.ClimberIOTalon
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.drivetrain.drive.DrivetrainIO
import com.team4099.robot2025.subsystems.drivetrain.drive.DrivetrainIOReal
import com.team4099.robot2025.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2025.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2025.subsystems.drivetrain.gyro.GyroIOPigeon2
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorIO
import com.team4099.robot2025.subsystems.elevator.ElevatorIOSim
import com.team4099.robot2025.subsystems.elevator.ElevatorIOTalon
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.limelight.LimelightVisionIO
import com.team4099.robot2025.subsystems.rollers.Rollers
import com.team4099.robot2025.subsystems.rollers.RollersIO
import com.team4099.robot2025.subsystems.rollers.RollersIOTalonFX
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.util.driver.Jessika
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.derived.Angle
import com.team4099.robot2025.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

object RobotContainer {
  private val drivetrain: Drivetrain
  private val limelight: LimelightVision
  private val arm: Arm
  private val climber: Climber
  private val elevator: Elevator
  private val rollers: Rollers
  val superstructure: Superstructure

  init {
    if (RobotBase.isReal()) {
      // Real Hardware Implementations
      // drivetrain = Drivetrain(object: GyroIO {},object: DrivetrainIO {}

      drivetrain = Drivetrain(object : GyroIO {}, object : DrivetrainIO {})
      limelight = LimelightVision(object : LimelightVisionIO {} )
      arm = Arm(object : ArmIO {})
      climber = Climber(object : ClimberIO {})
      elevator = Elevator(ElevatorIOTalon)
      rollers = Rollers(object : RollersIO {})
    } else {
      // Simulation implementations
      drivetrain = Drivetrain(object : GyroIO {}, DrivetrainIOSim)
      limelight = LimelightVision(object : LimelightVisionIO {})
      arm = Arm(ArmIOSim)
      climber = Climber(ClimberIOSim)
      elevator = Elevator(ElevatorIOSim)
      rollers = Rollers(RollersIOTalonFX)
    }

    superstructure = Superstructure(drivetrain, elevator, rollers, arm, climber, limelight)

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

  fun requestTuning() {
    superstructure.currentRequest = Request.SuperstructureRequest.Tuning()
  }

  fun mapTeleopControls() {

    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain))

    //tuning commands
    ControlBoard.testElevatorBind.whileTrue(superstructure.testElevatorCommand())
    ControlBoard.testElevatorDownBind.whileTrue(superstructure.testElevatorDownCommand())
    //ControlBoard.testClimberBind.whileTrue(superstructure.testClimberCommand())
    //ControlBoard.testRollersBind.whileTrue(superstructure.testRollersCommand())
    //ControlBoard.testArmBind.whileTrue(superstructure.testArmCommand())

//    ControlBoard.intakeCoral.whileTrue(superstructure.intakeCoralCommand())
//    ControlBoard.prepL2.whileTrue(superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L2))
//    ControlBoard.prepL3.whileTrue(superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L3))
//    ControlBoard.prepL4.whileTrue(superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4))
    //ControlBoard.score.onTrue(TestElevatorCommand(elevator))
    //ControlBoard.forceIdle.whileTrue(superstructure.requestIdleCommand())
  }

  fun mapTestControls() {}

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain)

  fun getAutonomousLoadingCommand() = AutonomousSelector.getLoadingCommand(drivetrain)

  fun resetGyroYawCommand(angle: Angle): Command = ResetGyroYawCommand(drivetrain, angle)

  fun mapTunableCommands() {}
}
