package com.team4099.robot2025.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.Constants.Universal.AlgaeLevel
import com.team4099.robot2025.config.constants.Constants.Universal.CoralLevel
import com.team4099.robot2025.config.constants.Constants.Universal.GamePiece
import com.team4099.robot2025.subsystems.arm.Arm
import com.team4099.robot2025.subsystems.arm.ArmTunableValues
import com.team4099.robot2025.subsystems.climber.Climber
import com.team4099.robot2025.subsystems.climber.ClimberTunableValues
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorTunableValues
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.rollers.Rollers
import com.team4099.robot2025.subsystems.rollers.RollersTunableValues
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.inMilliseconds

class Superstructure(
  private val drivetrain: Drivetrain,
  private val elevator: Elevator,
  private val rollers: Rollers,
  private val arm: Arm,
  private val climber: Climber,
  private val limelight: LimelightVision
) : SubsystemBase() {

  private var theoreticalGamePiece: GamePiece = GamePiece.NONE
  private var coralScoringLevel: CoralLevel = CoralLevel.NONE
  private var algaeIntakeLevel: AlgaeLevel = AlgaeLevel.NONE

  var currentRequest: Request.SuperstructureRequest = Request.SuperstructureRequest.Idle()
    set(value) {
      when (value) {
        is Request.SuperstructureRequest.IntakeAlgae -> {
          algaeIntakeLevel = value.level
        }
        is Request.SuperstructureRequest.ScorePrepCoral -> {
          coralScoringLevel = value.level
        }
        else -> {
          coralScoringLevel = CoralLevel.NONE
        }
      }
      field = value
    }

  private var currentState: SuperstructureStates = SuperstructureStates.UNINITIALIZED

  var isAtRequestedState: Boolean = false

  override fun periodic() {

    val armLoopStartTime = Clock.realTimestamp
    arm.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/ArmLoopTimeMS",
      (Clock.realTimestamp - armLoopStartTime).inMilliseconds
    )

    val climberLoopStartTime = Clock.realTimestamp
    climber.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/ClimberLoopTimeMS",
      (Clock.realTimestamp - climberLoopStartTime).inMilliseconds
    )

    val elevatorLoopStartTime = Clock.realTimestamp
    elevator.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/ElevatorLoopTimeMS",
      (Clock.realTimestamp - elevatorLoopStartTime).inMilliseconds
    )

    val rollersLoopStartTime = Clock.realTimestamp
    rollers.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/RollersLoopTimeMS",
      (Clock.realTimestamp - rollersLoopStartTime).inMilliseconds
    )

    CustomLogger.recordOutput("Superstructure/currentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Superstructure/currentState", currentState.name)
    CustomLogger.recordOutput("Superstructure/isAtAllTargetedPositions", isAtRequestedState)

    var nextState = currentState
    when (currentState) {
      SuperstructureStates.UNINITIALIZED -> {
        nextState = SuperstructureStates.HOME_PREP
      }
      SuperstructureStates.HOME_PREP -> {
        arm.currentRequest = Request.ArmRequest.Zero()

        if (arm.isZeroed) {
          nextState = SuperstructureStates.HOME
        }

        if (currentRequest is Request.SuperstructureRequest.Tuning) {
          nextState = SuperstructureStates.TUNING
        }
      }
      SuperstructureStates.HOME -> {
        elevator.currentRequest = Request.ElevatorRequest.Home()

        if (elevator.isHomed) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.TUNING -> {
        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.IDLE -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.idleVoltage.get())

        // mechanism idle positions based on which game piece robot has
        when (theoreticalGamePiece) {
          GamePiece.CORAL -> {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.idleCoralAngle.get())
            rollers.currentRequest =
              Request.RollersRequest.OpenLoop(RollersTunableValues.idleCoralVoltage.get())

            if (arm.isAtTargetedPosition)
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  ElevatorTunableValues.ElevatorHeights.idleCoralHeight.get()
                )
          }
          GamePiece.ALGAE -> {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.idleAlgaeAngle.get())
            rollers.currentRequest =
              Request.RollersRequest.OpenLoop(RollersTunableValues.idleAlgaeVoltage.get())

            if (arm.isAtTargetedPosition)
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  ElevatorTunableValues.ElevatorHeights.idleAlgaeHeight.get()
                )
          }
          GamePiece.NONE -> {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.idleAngle.get())
            rollers.currentRequest =
              Request.RollersRequest.OpenLoop(RollersTunableValues.idleVoltage.get())

            if (arm.isAtTargetedPosition)
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  ElevatorTunableValues.ElevatorHeights.idleHeight.get()
                )
          }
        }

        // idle to request transitions
        nextState =
          when (currentRequest) {
            is Request.SuperstructureRequest.Home -> SuperstructureStates.HOME_PREP
            is Request.SuperstructureRequest.IntakeAlgae -> SuperstructureStates.PREP_INTAKE_ALGAE
            is Request.SuperstructureRequest.IntakeCoral ->
              SuperstructureStates.PASS_THROUGH_TO_BACK
            is Request.SuperstructureRequest.ScorePrepCoral -> SuperstructureStates.SCORE_CORAL
            is Request.SuperstructureRequest.ScorePrepAlgaeProcessor ->
              SuperstructureStates.SCORE_ALGAE_PROCESSOR
            is Request.SuperstructureRequest.ScorePrepAlgaeBarge ->
              SuperstructureStates.SCORE_BARGE
            is Request.SuperstructureRequest.EjectGamepeice ->
              SuperstructureStates.PREP_EJECT_GAMEPIECE
            is Request.SuperstructureRequest.ClimbExtend -> SuperstructureStates.CLIMB_EXTEND
            is Request.SuperstructureRequest.ClimbRetract -> SuperstructureStates.CLIMB_RETRACT
            else -> currentState
          }
      }
      SuperstructureStates.PASS_THROUGH_TO_BACK -> {
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(
            ElevatorTunableValues.ElevatorHeights.passThroughHeight.get()
          )
        if (elevator.isAtTargetedPosition) {
          arm.currentRequest =
            Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.safeElevatorBackAngle.get())
          if (arm.isAtTargetedPosition) {
            nextState = SuperstructureStates.PREP_INTAKE_CORAL
          }
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.PASS_THROUGH_TO_FRONT
        }
      }
      SuperstructureStates.PREP_INTAKE_CORAL -> {
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(
            ElevatorTunableValues.ElevatorHeights.passThroughHeight.get()
          )
        if (elevator.isAtTargetedPosition) {
          arm.currentRequest =
            Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.intakeCoralAngle.get())
          if (arm.isAtTargetedPosition) {
            nextState = SuperstructureStates.INTAKE_CORAL
          }
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.CLEANUP_INTAKE_CORAL
        }
      }
      SuperstructureStates.INTAKE_CORAL -> {
        Request.RollersRequest.OpenLoop(RollersTunableValues.intakeCoralVoltage.get())

        if (rollers.hasCoral) {
          nextState = SuperstructureStates.CLEANUP_INTAKE_CORAL
          theoreticalGamePiece = GamePiece.CORAL
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.CLEANUP_INTAKE_CORAL
        }
      }
      SuperstructureStates.CLEANUP_INTAKE_CORAL -> {
        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.safeElevatorBackAngle.get())
        if (arm.isAtTargetedPosition) {
          elevator.currentRequest =
            Request.ElevatorRequest.ClosedLoop(
              ElevatorTunableValues.ElevatorHeights.passThroughHeight.get()
            )
          nextState = SuperstructureStates.PASS_THROUGH_TO_FRONT
        }
      }
      SuperstructureStates.PASS_THROUGH_TO_FRONT -> {
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(
            ElevatorTunableValues.ElevatorHeights.passThroughHeight.get()
          )
        if (elevator.isAtTargetedPosition) {
          arm.currentRequest =
            Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.safeElevatorFrontAngle.get())
          if (arm.isAtTargetedPosition) {
            currentRequest = Request.SuperstructureRequest.Idle()
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.PREP_SCORE_CORAL -> {
        when (coralScoringLevel) {
          CoralLevel.L1 -> {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.ElevatorHeights.L1Height.get()
              )
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                ArmTunableValues.ArmAngles.scoreCoralAngledAngle.get()
              )
          }
          CoralLevel.L2 -> {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.ElevatorHeights.L2Height.get()
              )
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                ArmTunableValues.ArmAngles.scoreCoralAngledAngle.get()
              )
          }
          CoralLevel.L3 -> {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.ElevatorHeights.L3Height.get()
              )
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                ArmTunableValues.ArmAngles.scoreCoralAngledAngle.get()
              )
          }
          CoralLevel.L4 -> {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.ElevatorHeights.L4Height.get()
              )
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                ArmTunableValues.ArmAngles.scoreCoralStraightAngle.get()
              )
          }
          else -> {}
        }
      }
      SuperstructureStates.SCORE_CORAL -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.intakeCoralVoltage.get())
      }
      SuperstructureStates.PREP_ELEVATOR_MOVEMENT -> {
        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.safeElevatorFrontAngle.get())
        if (arm.isAtTargetedPosition) {}

        // TODO:
        // if (currentRequest is Request.SuperstructureRequest.)
      }
      SuperstructureStates.PREP_INTAKE_ALGAE -> {
        when (algaeIntakeLevel) {
          AlgaeLevel.GROUND -> {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                ArmTunableValues.ArmAngles.safeElevatorFrontAngle.get()
              )

            if (arm.isAtTargetedPosition) {
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  ElevatorTunableValues.ElevatorHeights.intakeAlgaeGroundHeight.get()
                )
            }
          }
          AlgaeLevel.L2 -> {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                ArmTunableValues.ArmAngles.safeElevatorFrontAngle.get()
              )

            if (arm.isAtTargetedPosition) {
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  ElevatorTunableValues.ElevatorHeights.intakeAlgaeL2Height.get()
                )
            }
          }
          AlgaeLevel.L3 -> {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                ArmTunableValues.ArmAngles.safeElevatorFrontAngle.get()
              )

            if (arm.isAtTargetedPosition) {
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  ElevatorTunableValues.ElevatorHeights.intakeAlgaeL3Height.get()
                )
            }
          }
          else -> {}
        }

        if (arm.isAtTargetedPosition && elevator.isAtTargetedPosition) {
          nextState = SuperstructureStates.INTAKE_ALGAE
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.INTAKE_ALGAE -> {
        when (algaeIntakeLevel) {
          AlgaeLevel.GROUND -> {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                ArmTunableValues.ArmAngles.intakeAlgaeGroundAngle.get()
              )
          }
          AlgaeLevel.L2 -> {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.intakeAlgaeL2Angle.get())
          }
          AlgaeLevel.L3 -> {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.intakeAlgaeL3Angle.get())
          }
          else -> {}
        }

        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.intakeAlgaeVoltage.get())

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
          is Request.SuperstructureRequest.ScorePrepCoral -> {
            if (DriverStation.isAutonomous()) {
              nextState = SuperstructureStates.PREP_SCORE_CORAL
            }
          }
          is Request.SuperstructureRequest.ScorePrepAlgaeProcessor -> {
            if (DriverStation.isAutonomous()) {
              nextState = SuperstructureStates.PREP_SCORE_ALGAE_PROCESSOR
            }
          }
          is Request.SuperstructureRequest.ScorePrepAlgaeBarge -> {
            if (DriverStation.isAutonomous()) {
              nextState = SuperstructureStates.PREP_SCORE_BARGE
            }
          }
          else -> {}
        }
      }
      SuperstructureStates.PREP_SCORE_ALGAE_PROCESSOR -> {
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(
            ElevatorTunableValues.ElevatorHeights.scoreAlgaeProcessorHeight.get()
          )

        if (elevator.isAtTargetedPosition) {
          arm.currentRequest =
            Request.ArmRequest.ClosedLoop(
              ArmTunableValues.ArmAngles.scoreAlgaeProcessorAngle.get()
            )
        }
      }
      SuperstructureStates.SCORE_ALGAE_PROCESSOR -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.scoreProcessorAlgaeVoltage.get())
      }
      SuperstructureStates.PREP_SCORE_BARGE -> {
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(
            ElevatorTunableValues.ElevatorHeights.scoreAlgaeBargeHeight.get()
          )

        if (elevator.isAtTargetedPosition) {
          arm.currentRequest =
            Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.scoreAlgaeBargeAngle.get())
        }
      }
      SuperstructureStates.SCORE_BARGE -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.scoreBargeAlgaeVoltage.get())
      }
      SuperstructureStates.CLIMB_EXTEND -> {
        climber.currentRequest =
          Request.ClimberRequest.ClosedLoop(ClimberTunableValues.climbExtendAngle.get())
      }
      SuperstructureStates.CLIMB_RETRACT -> {
        climber.currentRequest =
          Request.ClimberRequest.ClosedLoop(ClimberTunableValues.climbRetractAngle.get())
      }
    }
    currentState = nextState
  }

  fun requestIdleCommand(): Command {
    val returnCommand =
      run { currentRequest = Request.SuperstructureRequest.Idle() }.until {
        isAtRequestedState && currentState == SuperstructureStates.IDLE
      }
    returnCommand.name = "RequestIdleCommand"
    return returnCommand
  }

  fun homeCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.Home() }.until {
        isAtRequestedState && currentState == SuperstructureStates.HOME
      }
    returnCommand.name = "HomeCommand"
    return returnCommand
  }

  fun climbExtendCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ClimbExtend() }.until {
        isAtRequestedState && currentState == SuperstructureStates.CLIMB_EXTEND
      }
    returnCommand.name = "ClimbExtendCommand"
    return returnCommand
  }

  fun climbRetractCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ClimbRetract() }.until {
        isAtRequestedState && currentState == SuperstructureStates.CLIMB_RETRACT
      }
    returnCommand.name = "ClimbRetractCommand"
    return returnCommand
  }

  fun prepScoreCoralL1Command(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ScorePrepCoral(CoralLevel.L1) }
        .until { isAtRequestedState && currentState == SuperstructureStates.PREP_SCORE_CORAL }
    returnCommand.name = "PrepCoralL1Command"
    return returnCommand
  }

  fun prepScoreCoralL2Command(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ScorePrepCoral(CoralLevel.L2) }
        .until { isAtRequestedState && currentState == SuperstructureStates.PREP_SCORE_CORAL }
    returnCommand.name = "PrepCoralL2Command"
    return returnCommand
  }

  fun prepScoreCoralL3Command(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ScorePrepCoral(CoralLevel.L3) }
        .until { isAtRequestedState && currentState == SuperstructureStates.PREP_SCORE_CORAL }
    returnCommand.name = "PrepCoralL3Command"
    return returnCommand
  }

  fun prepScoreCoralL4Command(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ScorePrepCoral(CoralLevel.L4) }
        .until { isAtRequestedState && currentState == SuperstructureStates.PREP_SCORE_CORAL }
    returnCommand.name = "PrepCoralL4Command"
    return returnCommand
  }

  fun testArmCommand(): Command {
    val returnCommand = runOnce {
      currentRequest = Request.SuperstructureRequest.Tuning()
      arm.currentRequest =
        Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.intakeCoralAngle.get())
    }
    returnCommand.name = "TestArmCommand"
    return returnCommand
  }

  fun testRollersCommand(): Command {
    val returnCommand = runOnce {
      currentRequest = Request.SuperstructureRequest.Tuning()
      rollers.currentRequest =
        Request.RollersRequest.OpenLoop(RollersTunableValues.intakeCoralVoltage.get())
    }
    returnCommand.name = "TestRollersCommand"
    return returnCommand
  }

  fun testElevatorCommand(): Command {
    val returnCommand = runOnce {
      currentRequest = Request.SuperstructureRequest.Tuning()
      elevator.currentRequest =
        Request.ElevatorRequest.ClosedLoop(
          ElevatorTunableValues.ElevatorHeights.testPosition.get()
        )
    }
    returnCommand.name = "TestElevatorCommand"
    return returnCommand
  }

  fun testClimberCommand(): Command {
    val returnCommand = runOnce {
      currentRequest = Request.SuperstructureRequest.Tuning()
      climber.currentRequest =
        Request.ClimberRequest.ClosedLoop(ClimberTunableValues.climbRetractAngle.get())
    }
    returnCommand.name = "TestClimberCommand"
    return returnCommand
  }

  companion object {
    enum class SuperstructureStates {
      UNINITIALIZED,
      TUNING,
      IDLE,
      HOME_PREP,
      HOME,
      PREP_INTAKE_CORAL,
      PASS_THROUGH_TO_BACK,
      PASS_THROUGH_TO_FRONT,
      INTAKE_CORAL,
      CLEANUP_INTAKE_CORAL,
      PREP_INTAKE_ALGAE,
      INTAKE_ALGAE,
      PREP_SCORE_ALGAE_PROCESSOR,
      SCORE_ALGAE_PROCESSOR,
      PREP_ELEVATOR_MOVEMENT,
      PREP_SCORE_CORAL,
      SCORE_CORAL,
      CLIMB_EXTEND,
      CLIMB_RETRACT,
      PREP_EJECT_GAMEPIECE,
      PREP_SCORE_BARGE,
      SCORE_BARGE,
    }
  }
}
