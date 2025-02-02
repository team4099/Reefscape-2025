package com.team4099.robot2025.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.Constants.Universal.AlgaeLevel
import com.team4099.robot2025.config.constants.Constants.Universal.CoralLevel
import com.team4099.robot2025.config.constants.Constants.Universal.GamePiece
import com.team4099.robot2025.config.constants.ElevatorConstants
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
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts

class Superstructure(
  private val drivetrain: Drivetrain,
  private val elevator: Elevator,
  private val rollers: Rollers,
  private val arm: Arm,
  private val climber: Climber,
  // private val climber: Climber,
  private val limelight: LimelightVision
) : SubsystemBase() {

  private var theoreticalGamePiece: GamePiece = GamePiece.NONE

  private var lastCoralScoringLevel: CoralLevel = CoralLevel.NONE
  private var coralScoringLevel: CoralLevel = CoralLevel.NONE

  private var lastAlgaeIntakeLevel: AlgaeLevel = AlgaeLevel.NONE
  private var algaeIntakeLevel: AlgaeLevel = AlgaeLevel.NONE

  var currentRequest: Request.SuperstructureRequest = Request.SuperstructureRequest.Tuning()
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

  private var lastTransitionTime = Clock.fpgaTime


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
    (Clock.realTimestamp - climberLoopStartTime).inMilliseconds)

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

    val elevatorPosition = elevator.inputs.elevatorPosition
    //Firsts Stage
    CustomLogger.recordOutput("SimulatedMechanisms/0",
      Pose3d().transformBy(
        Transform3d(
          Translation3d(0.0.inches, 0.0.inches,
            if (elevatorPosition > ElevatorConstants.SECOND_STAGE_HEIGHT) elevatorPosition - ElevatorConstants.SECOND_STAGE_HEIGHT
            else 0.0.inches),
          Rotation3d()
        )
      ).pose3d)

       //Second Stage
    CustomLogger.recordOutput("SimulatedMechanisms/1",
      Pose3d().transformBy(
        Transform3d(
          Translation3d(0.0.inches, 0.0.inches,
            if (elevatorPosition > ElevatorConstants.FIRST_STAGE_HEIGHT) elevatorPosition - ElevatorConstants.FIRST_STAGE_HEIGHT
            else 0.0.inches),
          Rotation3d()
        )
      ).pose3d)

    //Carriage
    CustomLogger.recordOutput("SimulatedMechanisms/2",
      Pose3d().transformBy(
        Transform3d(
          Translation3d(0.0.inches, 0.0.inches, elevatorPosition),
          Rotation3d()
        )
      ).pose3d)

    //Arm
    CustomLogger.recordOutput("SimulatedMechanisms/3",
      Pose3d().transformBy(
        Transform3d(
          Translation3d(8.5.inches, 0.0.inches, 15.3125.inches + elevatorPosition),
          Rotation3d(0.0.degrees, -arm.inputs.armPosition, 0.0.degrees)
        )
      ).pose3d)


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
      }
      SuperstructureStates.HOME -> {
        elevator.currentRequest = Request.ElevatorRequest.Home()

        if (elevator.isHomed) {
          elevator.currentRequest =  Request.ElevatorRequest.OpenLoop(0.0.volts)
          nextState = SuperstructureStates.TUNING
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

            if (arm.isAtTargetedPosition) {
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  ElevatorTunableValues.ElevatorHeights.idleCoralHeight.get()
                )
            }
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
            is Request.SuperstructureRequest.IntakeAlgae ->
              SuperstructureStates.PREP_ELEVATOR_MOVEMENT
            is Request.SuperstructureRequest.IntakeCoral ->
              SuperstructureStates.PREP_INTAKE_CORAL
            is Request.SuperstructureRequest.ScorePrepCoral ->
              SuperstructureStates.PREP_ELEVATOR_MOVEMENT
            is Request.SuperstructureRequest.ScorePrepAlgaeProcessor ->
              SuperstructureStates.SCORE_ALGAE_PROCESSOR
            is Request.SuperstructureRequest.ScorePrepAlgaeBarge ->
              SuperstructureStates.PREP_ELEVATOR_MOVEMENT
            is Request.SuperstructureRequest.EjectGamepeice ->
              SuperstructureStates.PREP_EJECT_GAMEPIECE
            is Request.SuperstructureRequest.ClimbExtend -> SuperstructureStates.CLIMB_EXTEND
            is Request.SuperstructureRequest.ClimbRetract -> SuperstructureStates.CLIMB_RETRACT
            is Request.SuperstructureRequest.Tuning -> SuperstructureStates.TUNING
            else -> currentState
          }
      }
      SuperstructureStates.PREP_INTAKE_CORAL -> {

        val elevatorInSafeState = elevator.inputs.elevatorPosition > ElevatorTunableValues.ElevatorHeights.safeArmPassUnderCarriageHeight.get()
        val armInSafeState = arm.inputs.armPosition > ArmTunableValues.ArmAngles.safeElevatorBackAngle.get()

        if (!elevatorInSafeState) {
          arm.currentRequest = Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.safeElevatorBackAngle.get())
        } else {
          arm.currentRequest = Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.intakeCoralAngle.get())
        }

        if (!armInSafeState) {
          elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.ElevatorHeights.safeArmPassUnderCarriageHeight.get())
        } else {
          elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.ElevatorHeights.intakeCoralHeight.get())
        }

        if (armInSafeState && elevatorInSafeState && arm.isAtTargetedPosition && elevator.isAtTargetedPosition) {
          nextState = SuperstructureStates.INTAKE_CORAL
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.INTAKE_CORAL_CLEANUP
        }
      }

      SuperstructureStates.INTAKE_CORAL -> {
        Request.RollersRequest.OpenLoop(RollersTunableValues.intakeCoralVoltage.get())

        if (rollers.hasCoral) {
          nextState = SuperstructureStates.INTAKE_CORAL_CLEANUP
          theoreticalGamePiece = GamePiece.CORAL
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.INTAKE_CORAL_CLEANUP
        }
      }

      SuperstructureStates.INTAKE_CORAL_CLEANUP -> {
        rollers.currentRequest = Request.RollersRequest.OpenLoop(RollersTunableValues.idleCoralVoltage.get())

        val elevatorInSafeState = elevator.inputs.elevatorPosition < ElevatorTunableValues.ElevatorHeights.safeArmPassThroughCarriageHeight.get()
        val armInSafeState = arm.inputs.armPosition < ArmTunableValues.ArmAngles.safeElevatorBackAngle.get()

        if (!armInSafeState) {
          elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.ElevatorHeights.safeArmPassUnderCarriageHeight.get())
        } else {
          elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(
            if (theoreticalGamePiece == GamePiece.CORAL) ElevatorTunableValues.ElevatorHeights.idleCoralHeight.get()
            else ElevatorTunableValues.ElevatorHeights.idleHeight.get()
          )
        }

        if (!elevatorInSafeState) {
          arm.currentRequest = Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.safeElevatorBackAngle.get())
        } else {
          arm.currentRequest = Request.ArmRequest.ClosedLoop(
            if (theoreticalGamePiece == GamePiece.CORAL) ArmTunableValues.ArmAngles.idleCoralAngle.get()
            else ArmTunableValues.ArmAngles.idleAngle.get()
          )
        }

        if (armInSafeState && elevatorInSafeState && arm.isAtTargetedPosition && elevator.isAtTargetedPosition) {
          nextState = SuperstructureStates.IDLE
        }

      }

      SuperstructureStates.PREP_ELEVATOR_MOVEMENT -> {
        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.safeElevatorFrontAngle.get())
        if (arm.isAtTargetedPosition) {
          when (currentRequest) {
            is Request.SuperstructureRequest.ScorePrepCoral -> {
              nextState = SuperstructureStates.PREP_SCORE_CORAL
            }
            is Request.SuperstructureRequest.IntakeAlgae -> {
              nextState = SuperstructureStates.PREP_INTAKE_ALGAE
            }
            is Request.SuperstructureRequest.ScorePrepAlgaeBarge -> {
              nextState = SuperstructureStates.PREP_SCORE_ALGAE_BARGE
            }
            is Request.SuperstructureRequest.Idle -> {
              nextState = SuperstructureStates.IDLE
            }
          }
        }
      }
      SuperstructureStates.PREP_SCORE_CORAL -> {

        when (currentRequest) {
          is Request.SuperstructureRequest.ScorePrepCoral -> {
            if (coralScoringLevel != lastCoralScoringLevel) {
              nextState = SuperstructureStates.PREP_ELEVATOR_MOVEMENT
            } else {
              val reefLevelElevatorHeight: Length =
                when (coralScoringLevel) {
                  CoralLevel.L1 -> ElevatorTunableValues.ElevatorHeights.L1Height.get()
                  CoralLevel.L2 -> ElevatorTunableValues.ElevatorHeights.L2Height.get()
                  CoralLevel.L3 -> ElevatorTunableValues.ElevatorHeights.L3Height.get()
                  CoralLevel.L4 -> ElevatorTunableValues.ElevatorHeights.L4Height.get()
                  else -> 0.0.inches
                }

              val reefLevelArmAngle: Angle =
                when (coralScoringLevel) {
                  CoralLevel.L1 -> ArmTunableValues.ArmAngles.scoreCoralL1Angle.get()
                  CoralLevel.L2 -> ArmTunableValues.ArmAngles.scoreCoralL2Angle.get()
                  CoralLevel.L3 -> ArmTunableValues.ArmAngles.scoreCoralL3Angle.get()
                  CoralLevel.L4 -> ArmTunableValues.ArmAngles.scoreCoralL4Angle.get()
                  else -> {
                    0.0.degrees
                  }
                }

              elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(reefLevelElevatorHeight)
              if (elevator.isAtTargetedPosition) {
                arm.currentRequest = Request.ArmRequest.ClosedLoop(reefLevelArmAngle)
              }
            }
          }
          is Request.SuperstructureRequest.Score -> {
            if (arm.isAtTargetedPosition && elevator.isAtTargetedPosition) {
              nextState = SuperstructureStates.SCORE_CORAL
            }
          }
          is Request.SuperstructureRequest.IntakeAlgae -> {
            nextState = SuperstructureStates.PREP_ELEVATOR_MOVEMENT
          }
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.SCORE_CLEANUP
          }
        }
      }
      SuperstructureStates.SCORE_CORAL -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.scoreCoralVoltage.get())

        if ((Clock.fpgaTime - lastTransitionTime) > RollersTunableValues.coralSpitTime.get()) {
          theoreticalGamePiece = GamePiece.NONE
          nextState = SuperstructureStates.SCORE_CLEANUP
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.SCORE_CLEANUP
        }
      }
      SuperstructureStates.SCORE_CLEANUP -> {
        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.safeElevatorFrontAngle.get())
        if (arm.isAtTargetedPosition) {
          elevator.currentRequest =
            Request.ElevatorRequest.ClosedLoop(
              ElevatorTunableValues.ElevatorHeights.idleHeight.get()
            )
          if (elevator.isAtTargetedPosition) {
            currentRequest = Request.SuperstructureRequest.Idle()
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.PREP_INTAKE_ALGAE -> {

        when(currentRequest) {
          is Request.SuperstructureRequest.IntakeAlgae -> {
            if (algaeIntakeLevel != lastAlgaeIntakeLevel) {
              nextState = SuperstructureStates.PREP_ELEVATOR_MOVEMENT
            } else {
              val algaeIntakeElevatorHeight: Length =
                when (algaeIntakeLevel) {
                  AlgaeLevel.GROUND ->
                    ElevatorTunableValues.ElevatorHeights.intakeAlgaeGroundHeight.get()

                  AlgaeLevel.L2 -> ElevatorTunableValues.ElevatorHeights.intakeAlgaeL2Height.get()
                  AlgaeLevel.L3 -> ElevatorTunableValues.ElevatorHeights.intakeAlgaeL3Height.get()
                  else -> 0.0.inches
                }

              val algaeIntakeArmAngle: Angle =
                when (algaeIntakeLevel) {
                  AlgaeLevel.GROUND -> ArmTunableValues.ArmAngles.intakeAlgaeGroundAngle.get()
                  AlgaeLevel.L2 -> ArmTunableValues.ArmAngles.intakeAlgaeL2Angle.get()
                  AlgaeLevel.L3 -> ArmTunableValues.ArmAngles.intakeAlgaeL3Angle.get()
                  else -> 0.0.degrees
                }

              elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(algaeIntakeElevatorHeight)

              if (elevator.isAtTargetedPosition) {
                arm.currentRequest = Request.ArmRequest.ClosedLoop(algaeIntakeArmAngle)
                if (arm.isAtTargetedPosition) {
                  nextState = SuperstructureStates.INTAKE_ALGAE
                }
              }
            }
          }
          is Request.SuperstructureRequest.ScorePrepCoral -> {
            nextState = SuperstructureStates.PREP_ELEVATOR_MOVEMENT
          }
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.INTAKE_ALGAE -> {

        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.intakeAlgaeVoltage.get())

        if (rollers.hasAlgae) {
          nextState = SuperstructureStates.INTAKE_ALGAE_CLEANUP
          theoreticalGamePiece = GamePiece.ALGAE
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.INTAKE_ALGAE_CLEANUP
        }
      }
      SuperstructureStates.INTAKE_ALGAE_CLEANUP -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.idleAlgaeVoltage.get())
        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.safeElevatorFrontAngle.get())
        if (arm.isAtTargetedPosition) {
          elevator.currentRequest =
            Request.ElevatorRequest.ClosedLoop(
              ElevatorTunableValues.ElevatorHeights.idleAlgaeHeight.get()
            )
          if (elevator.isAtTargetedPosition) {
            currentRequest = Request.SuperstructureRequest.Idle()
            nextState = SuperstructureStates.IDLE
          }
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
          if (arm.isAtTargetedPosition && currentRequest is Request.SuperstructureRequest.Score) {
            nextState = SuperstructureStates.SCORE_ALGAE_PROCESSOR
          }
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.SCORE_ALGAE_PROCESSOR -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.scoreProcessorAlgaeVoltage.get())

        if ((Clock.fpgaTime - lastTransitionTime) >
          RollersTunableValues.algaeProcessorSpitTime.get()
        ) {
          theoreticalGamePiece = GamePiece.NONE
          nextState = SuperstructureStates.IDLE
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.PREP_SCORE_ALGAE_BARGE -> {
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(
            ElevatorTunableValues.ElevatorHeights.scoreAlgaeBargeHeight.get()
          )

        if (elevator.isAtTargetedPosition) {
          arm.currentRequest =
            Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.scoreAlgaeBargeAngle.get())
          if (arm.isAtTargetedPosition && currentRequest is Request.SuperstructureRequest.Score) {
            nextState = SuperstructureStates.SCORE_ALGAE_BARGE
          }
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.SCORE_CLEANUP
        }
      }
      SuperstructureStates.SCORE_ALGAE_BARGE -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.scoreBargeAlgaeVoltage.get())

        if ((Clock.fpgaTime - lastTransitionTime) > RollersTunableValues.algaeBargeSpitTime.get()) {
          theoreticalGamePiece = GamePiece.NONE
          nextState = SuperstructureStates.SCORE_CLEANUP
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.SCORE_CLEANUP
        }
      }
      SuperstructureStates.PREP_EJECT_GAMEPIECE -> {
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(
            ElevatorTunableValues.ElevatorHeights.ejectHeight.get()
          )

        if (elevator.isAtTargetedPosition)
          arm.currentRequest =
            Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.ejectAngle.get())
      }
      SuperstructureStates.EJECT_GAMEPIECE -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.ejectVoltage.get())
      }
      SuperstructureStates.CLEANUP_EJECT_GAMEPIECE -> {
        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.idleAngle.get())
      }
      //      SuperstructureStates.CLIMB_EXTEND -> {
      //        climber.currentRequest =
      //          Request.ClimberRequest.ClosedLoop(ClimberTunableValues.climbExtendAngle.get())
      //
      //        if (climber.isAtTargetedPosition &&
      //          currentRequest is Request.SuperstructureRequest.ClimbRetract
      //        ) {
      //          currentState = SuperstructureStates.CLIMB_RETRACT
      //        }
      //        if (currentRequest is Request.SuperstructureRequest.Idle) {
      //          currentState = SuperstructureStates.IDLE
      //        }
      //      }
      //      SuperstructureStates.CLIMB_RETRACT -> {
      //        climber.currentRequest =
      //          Request.ClimberRequest.ClosedLoop(ClimberTunableValues.climbRetractAngle.get())
      //
      //        if (currentRequest is Request.SuperstructureRequest.Idle) {
      //          currentState = SuperstructureStates.IDLE
      //        }
      //      }
    }

    if (nextState != currentState) {
      lastTransitionTime = Clock.fpgaTime
    }

    lastCoralScoringLevel = coralScoringLevel
    lastAlgaeIntakeLevel = algaeIntakeLevel

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

  fun intakeCoralCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.IntakeCoral() }.until {
        isAtRequestedState && currentState == SuperstructureStates.PREP_INTAKE_CORAL
      }
    returnCommand.name = "IntakeCoralCommand"
    return returnCommand
  }

  fun intakeAlgaeCommand(level: AlgaeLevel): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.IntakeAlgae(level) }.until {
        isAtRequestedState && currentState == SuperstructureStates.PREP_INTAKE_ALGAE
      }
    returnCommand.name = "IntakeAlgaeCommand"
    return returnCommand
  }

  fun prepScoreAlgaeBargeCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ScorePrepAlgaeBarge() }.until {
        isAtRequestedState && currentState == SuperstructureStates.PREP_SCORE_ALGAE_BARGE
      }
    returnCommand.name = "PrepScoreAlgaeBargeCommand"
    return returnCommand
  }

  fun prepScoreAlgaeProcessorCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ScorePrepAlgaeProcessor() }.until {
        isAtRequestedState && currentState == SuperstructureStates.PREP_SCORE_ALGAE_PROCESSOR
      }
    returnCommand.name = "PrepScoreAlgaeProcessorCommand"
    return returnCommand
  }


  fun prepScoreCoralCommand(level: CoralLevel): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ScorePrepCoral(level) }.until {
        isAtRequestedState && currentState == SuperstructureStates.PREP_SCORE_CORAL
      }
    returnCommand.name = "PrepScoreCoralCommand"
    return returnCommand
  }

  fun scoreCommand(): Command {
    val returnCommand =
      run {
        if (currentState == SuperstructureStates.IDLE || currentState == SuperstructureStates.PREP_ELEVATOR_MOVEMENT) {
          currentRequest = Request.SuperstructureRequest.ScorePrepCoral(CoralLevel.L1)
        } else  {
          currentRequest = Request.SuperstructureRequest.Score()
        }
        }.until {
       (currentState == SuperstructureStates.SCORE_CORAL ||
                currentState == SuperstructureStates.SCORE_ALGAE_PROCESSOR ||
                currentState == SuperstructureStates.SCORE_ALGAE_BARGE)
      }
    returnCommand.name = "ScoreCommand"
    return returnCommand
  }

  fun ejectGamePieceCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.EjectGamepeice() }
    returnCommand.name = "EjectGamePiece"
    return returnCommand
  }

  fun testArmCommand(): Command {
    val returnCommand = run {
      currentRequest = Request.SuperstructureRequest.Tuning()
      arm.currentRequest = Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.testAngle.get())

      if (arm.isAtTargetedPosition) {
        currentRequest = Request.SuperstructureRequest.Idle()
      }
    }
    returnCommand.name = "TestArmCommand"
    return returnCommand
  }

  fun testRollersCommand(): Command {
    val returnCommand = run {
      currentRequest = Request.SuperstructureRequest.Tuning()
      rollers.currentRequest =
        Request.RollersRequest.OpenLoop(RollersTunableValues.intakeCoralVoltage.get())
    }
    returnCommand.name = "TestRollersCommand"
    return returnCommand
  }

  fun testElevatorCommand(): Command {
    val returnCommand = run {
      currentRequest = Request.SuperstructureRequest.Tuning()
      elevator.currentRequest =
        Request.ElevatorRequest.ClosedLoop(
          ElevatorTunableValues.ElevatorHeights.testPosition.get()
        )

      if (elevator.isAtTargetedPosition) {
        currentRequest = Request.SuperstructureRequest.Idle()
      }
    }
    returnCommand.name = "TestElevatorCommand"
    return returnCommand
  }

  fun testElevatorDownCommand(): Command {
    val returnCommand = run {
      currentRequest = Request.SuperstructureRequest.Tuning()
      elevator.currentRequest =
        Request.ElevatorRequest.ClosedLoop(
          5.inches
        )

      if (elevator.isAtTargetedPosition) {
        currentRequest = Request.SuperstructureRequest.Idle()
      }
    }
    returnCommand.name = "TestElevatorDownCommand"
    return returnCommand
  }

  fun testClimberCommand(): Command {
      val returnCommand = run {
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
      INTAKE_CORAL,
      INTAKE_CORAL_CLEANUP,
      PREP_INTAKE_ALGAE,
      INTAKE_ALGAE,
      INTAKE_ALGAE_CLEANUP,
      PREP_SCORE_ALGAE_PROCESSOR,
      SCORE_ALGAE_PROCESSOR,
      PREP_ELEVATOR_MOVEMENT,
      PREP_SCORE_CORAL,
      SCORE_CORAL,
      SCORE_CLEANUP,
      CLIMB_EXTEND,
      CLIMB_RETRACT,
      PREP_EJECT_GAMEPIECE,
      EJECT_GAMEPIECE,
      CLEANUP_EJECT_GAMEPIECE,
      PREP_SCORE_ALGAE_BARGE,
      SCORE_ALGAE_BARGE,
    }
  }
}
