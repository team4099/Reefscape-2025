package com.team4099.robot2025.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.subsystems.led.Leds
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.config.constants.Constants.Universal.AlgaeLevel
import com.team4099.robot2025.config.constants.Constants.Universal.CoralLevel
import com.team4099.robot2025.config.constants.Constants.Universal.GamePiece
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.subsystems.arm.Arm
import com.team4099.robot2025.subsystems.arm.ArmTunableValues
import com.team4099.robot2025.subsystems.climber.Climber
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorTunableValues
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.rollers.Ramp
import com.team4099.robot2025.subsystems.rollers.RampTunableValues
import com.team4099.robot2025.subsystems.rollers.Rollers
import com.team4099.robot2025.subsystems.rollers.RollersTunableValues
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.milli
import org.team4099.lib.units.perSecond

class Superstructure(
  private val drivetrain: Drivetrain,
  private val elevator: Elevator,
  private val rollers: Rollers,
  private val ramp: Ramp,
  private val arm: Arm,
  private val climber: Climber,
  private val leds: Leds,
  private val vision: Vision,
  private val limelight: LimelightVision
) : SubsystemBase() {
  var theoreticalGamePiece: GamePiece = GamePiece.NONE
    get() {
      if (rollers.hasCoral || ramp.hasCoral) {
        return GamePiece.CORAL
      } else {
        return field
      }
    }

  private var lastCoralScoringLevel: CoralLevel = CoralLevel.NONE
  private var coralScoringLevel: CoralLevel = CoralLevel.NONE

  private var lastAlgaeIntakeLevel: AlgaeLevel = AlgaeLevel.NONE
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
        is Request.SuperstructureRequest.Score -> {
          coralScoringLevel = coralScoringLevel
        }
        else -> {
          coralScoringLevel = CoralLevel.NONE
        }
      }
      field = value
    }

  var currentState: SuperstructureStates = SuperstructureStates.UNINITIALIZED

  var isAtRequestedState: Boolean = false

  private var lastTransitionTime = Clock.fpgaTime

  override fun periodic() {
    // led updates
    leds.hasCoral = theoreticalGamePiece == GamePiece.CORAL
    leds.isAutoAligning = vision.isAutoAligning
    leds.isAligned = vision.isAligned
    leds.seesTag = Clock.fpgaTime - vision.lastTrigVisionUpdate.timestamp < 500.milli.seconds
    leds.isIntaking =
      currentState == SuperstructureStates.INTAKE_CORAL ||
      currentState == SuperstructureStates.PREP_INTAKE_CORAL ||
      currentState == SuperstructureStates.INTAKE_ALGAE ||
      currentState == SuperstructureStates.PREP_INTAKE_ALGAE

    leds.closestReefTagID = vision.lastTrigVisionUpdate.targetTagID

    val ledLoopStartTime = Clock.realTimestamp
    leds.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/ledLoopTimeMS",
      (Clock.realTimestamp - ledLoopStartTime).inMilliseconds
    )

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

    val rampLoopStartTime = Clock.realTimestamp
    ramp.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/RampLoopTimeMS",
      (Clock.realTimestamp - rampLoopStartTime).inMilliseconds
    )

    CustomLogger.recordOutput("Superstructure/currentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Superstructure/currentState", currentState.name)
    CustomLogger.recordOutput("Superstructure/isAtAllTargetedPositions", isAtRequestedState)
    CustomLogger.recordOutput("Superstructure/theoreticalGamePiece", theoreticalGamePiece)

    val elevatorPosition = elevator.inputs.elevatorPosition

    // First Elevator Stage
    CustomLogger.recordDebugOutput(
      "SimulatedMechanisms/0",
      Pose3d()
        .transformBy(
          Transform3d(
            Translation3d(
              0.0.inches,
              0.0.inches,
              if (elevatorPosition > ElevatorConstants.SECOND_STAGE_HEIGHT)
                elevatorPosition - ElevatorConstants.SECOND_STAGE_HEIGHT
              else 0.0.inches
            ),
            Rotation3d()
          )
        )
        .pose3d
    )

    // Second Elevator Stage
    CustomLogger.recordDebugOutput(
      "SimulatedMechanisms/1",
      Pose3d()
        .transformBy(
          Transform3d(
            Translation3d(
              0.0.inches,
              0.0.inches,
              if (elevatorPosition > ElevatorConstants.FIRST_STAGE_HEIGHT)
                elevatorPosition - ElevatorConstants.FIRST_STAGE_HEIGHT
              else 0.0.inches
            ),
            Rotation3d()
          )
        )
        .pose3d
    )

    // Carriage
    CustomLogger.recordDebugOutput(
      "SimulatedMechanisms/2",
      Pose3d()
        .transformBy(
          Transform3d(Translation3d(0.0.inches, 0.0.inches, elevatorPosition), Rotation3d())
        )
        .pose3d
    )

    var nextState = currentState
    when (currentState) {
      // General States
      SuperstructureStates.UNINITIALIZED -> {
        nextState = SuperstructureStates.HOME_PREP
      }
      SuperstructureStates.TUNING -> {
        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.MANUAL_RESET -> {
        arm.currentRequest = Request.ArmRequest.TorqueControl(-ArmTunableValues.ArmCurrents.stallCurrent.get())
        elevator.currentRequest = Request.ElevatorRequest.Home()
      }
      SuperstructureStates.EJECT -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.ejectVoltage.get())
        ramp.currentRequest = Request.RampRequest.OpenLoop(RampTunableValues.ejectVoltage.get())

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
          is Request.SuperstructureRequest.IntakeCoral -> {
            nextState = SuperstructureStates.INTAKE_CORAL
          }
        }
      }
      SuperstructureStates.HOME_PREP -> {
        nextState = SuperstructureStates.HOME
      }
      SuperstructureStates.HOME -> {
        elevator.currentRequest = Request.ElevatorRequest.Home()
      }
      SuperstructureStates.IDLE -> {
        climber.currentRequest = Request.ClimberRequest.OpenLoop(0.0.volts)
        ramp.currentRequest = Request.RampRequest.OpenLoop(RampTunableValues.idleVoltage.get())

        if (elevator.inputs.leaderStatorCurrent > ElevatorConstants.HOMING_STALL_CURRENT &&
          elevator.elevatorVelocityTarget == 0.inches.perSecond &&
          elevator.elevatorPositionTarget == 0.inches &&
          elevator.inputs.elevatorPosition - elevator.elevatorPositionTarget <=
          ElevatorConstants.ELEVATOR_TOLERANCE &&
          elevator.inputs.leaderAppliedVoltage < 0.volts &&
          (Clock.fpgaTime - lastTransitionTime) >
          0.5.seconds
        ) { // add buffer so it won't home when still up
          elevator.currentRequest = Request.ElevatorRequest.Home() // stop stalling
        } else {
          elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(0.0.inches)
        }

        when (theoreticalGamePiece) {
          GamePiece.CORAL -> {
            rollers.currentRequest =
              Request.RollersRequest.OpenLoop(RollersTunableValues.idleCoralVoltage.get())
          }
          GamePiece.ALGAE -> {
            rollers.currentRequest =
              Request.RollersRequest.OpenLoop(RollersTunableValues.idleAlgaeVoltage.get())
          }
          GamePiece.NONE -> {
            rollers.currentRequest =
              Request.RollersRequest.OpenLoop(RollersTunableValues.idleVoltage.get())
          }
        }

        // idle to request transitions
        nextState =
          when (currentRequest) {
            is Request.SuperstructureRequest.Home -> SuperstructureStates.HOME_PREP
            is Request.SuperstructureRequest.IntakeAlgae -> SuperstructureStates.PREP_INTAKE_ALGAE
            is Request.SuperstructureRequest.IntakeCoral -> SuperstructureStates.PREP_INTAKE_CORAL
            is Request.SuperstructureRequest.ScorePrepCoral ->
              SuperstructureStates.PREP_SCORE_CORAL
            is Request.SuperstructureRequest.ScorePrepAlgaeProcessor ->
              SuperstructureStates.PREP_SCORE_ALGAE_PROCESSOR
            is Request.SuperstructureRequest.ScorePrepAlgaeBarge ->
              SuperstructureStates.PREP_SCORE_ALGAE_BARGE
            is Request.SuperstructureRequest.ClimbExtend -> SuperstructureStates.CLIMB_EXTEND
            is Request.SuperstructureRequest.ClimbRetract -> SuperstructureStates.CLIMB_RETRACT
            is Request.SuperstructureRequest.Tuning -> SuperstructureStates.TUNING
            else -> currentState
          }
      }

      // Coral States
      SuperstructureStates.PREP_INTAKE_CORAL -> {
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(
            ElevatorTunableValues.ElevatorHeights.intakeCoralHeight.get()
          )

        if (elevator.isAtTargetedPosition) {
          nextState = SuperstructureStates.INTAKE_CORAL
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.INTAKE_CORAL -> {
        ramp.currentRequest =
          Request.RampRequest.OpenLoop(RampTunableValues.intakeCoralVoltageFast.get())
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.intakeCoralVoltageFast.get())

        if (rollers.hasCoral) {
          theoreticalGamePiece = GamePiece.CORAL
          currentRequest = Request.SuperstructureRequest.Idle()
        }

        if (currentRequest is Request.SuperstructureRequest.Idle ||
          currentRequest is Request.SuperstructureRequest.ScorePrepCoral
        ) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.PREP_SCORE_CORAL -> {
        elevator.currentRequest =
          when (coralScoringLevel) {
            CoralLevel.L1 ->
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.ElevatorHeights.L1Height.get()
              )
            CoralLevel.L2 ->
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.ElevatorHeights.L2Height.get()
              )
            CoralLevel.L3 ->
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.ElevatorHeights.L3Height.get()
              )
            CoralLevel.L4 ->
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.ElevatorHeights.PrepL4Height.get()
              )
            else ->
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.ElevatorHeights.idleHeight.get()
              )
          }

        when (currentRequest) {
          is Request.SuperstructureRequest.Score -> {
            if (elevator.isAtTargetedPosition) {
              nextState = SuperstructureStates.SCORE_CORAL
            }
          }
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_CORAL -> {
        if (theoreticalGamePiece == GamePiece.CORAL || theoreticalGamePiece == GamePiece.NONE) {
          if (coralScoringLevel == CoralLevel.L4) {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.ElevatorHeights.L4Height.get()
              )
            if (elevator.isAtTargetedPosition &&
              elevator.inputs.elevatorPosition >=
              ElevatorTunableValues.ElevatorHeights.L4HeightToScore.get()
            ) {
              rollers.currentRequest =
                Request.RollersRequest.OpenLoop(RollersTunableValues.scoreCoralVoltage.get())
            }
          } else {
            if (elevator.isAtTargetedPosition) {
              rollers.currentRequest =
                Request.RollersRequest.OpenLoop(RollersTunableValues.scoreCoralVoltage.get())
            }
          }

          val spitOutTime =
            if (coralScoringLevel == CoralLevel.L4) (RollersTunableValues.coralL4SpitTime.get())
            else RollersTunableValues.coralSpitTime.get()

          if ((Clock.fpgaTime - lastTransitionTime) > spitOutTime) {
            theoreticalGamePiece = GamePiece.NONE
            nextState = SuperstructureStates.IDLE
          }
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }

      // Algae States
      // TODO: Once design is finalized, recheck and rewrite algae code if necessary
      SuperstructureStates.PREP_INTAKE_ALGAE -> {
        if (algaeIntakeLevel != lastAlgaeIntakeLevel) {
          nextState = SuperstructureStates.INTAKE_ALGAE
        } else {
          val algaeIntakeElevatorHeight: Length =
            when (algaeIntakeLevel) {
              AlgaeLevel.GROUND ->
                ElevatorTunableValues.ElevatorHeights.intakeAlgaeGroundHeight.get()
              AlgaeLevel.L2 -> ElevatorTunableValues.ElevatorHeights.intakeAlgaeL2Height.get()
              AlgaeLevel.L3 -> ElevatorTunableValues.ElevatorHeights.intakeAlgaeL3Height.get()
              else -> 0.0.inches
            }

          elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(algaeIntakeElevatorHeight)

          if (elevator.isAtTargetedPosition) {
            arm.currentRequest =
              Request.ArmRequest.TorqueControl(ArmTunableValues.ArmCurrents.stallCurrent.get())
          }
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.ScorePrepAlgaeBarge -> {
            if (elevator.isAtTargetedPosition) {
              nextState = SuperstructureStates.SCORE_ALGAE_BARGE
            }
          }
          is Request.SuperstructureRequest.ScorePrepAlgaeProcessor -> {
            if (elevator.isAtTargetedPosition) {
              nextState = SuperstructureStates.SCORE_ALGAE_PROCESSOR
            }
          }
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.INTAKE_ALGAE -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.intakeAlgaeVoltage.get())

        if (currentRequest is Request.SuperstructureRequest.Idle ||
          currentRequest is Request.SuperstructureRequest.IntakeCoral
        ) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.PREP_SCORE_ALGAE_PROCESSOR -> {
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(
            ElevatorTunableValues.ElevatorHeights.scoreAlgaeProcessorHeight.get()
          )
        if (elevator.isAtTargetedPosition) {
          arm.currentRequest =
            Request.ArmRequest.TorqueControl(ArmTunableValues.ArmCurrents.stallCurrent.get())
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Score -> {
            if (elevator.isAtTargetedPosition && arm.isAtTargetedPosition) {
              nextState = SuperstructureStates.SCORE_ALGAE_PROCESSOR
            }
          }
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
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
            Request.ArmRequest.TorqueControl(ArmTunableValues.ArmCurrents.stallCurrent.get())

          if (arm.isAtTargetedPosition && currentRequest is Request.SuperstructureRequest.Score) {
            nextState = SuperstructureStates.SCORE_ALGAE_BARGE
          }
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.SCORE_ALGAE_BARGE -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.scoreBargeAlgaeVoltage.get())

        if ((Clock.fpgaTime - lastTransitionTime) > RollersTunableValues.algaeBargeSpitTime.get()) {
          theoreticalGamePiece = GamePiece.NONE
          nextState = SuperstructureStates.IDLE
        }
        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }

      // Climber States
      SuperstructureStates.CLIMB_EXTEND -> {
        climber.currentRequest =
          Request.ClimberRequest.OpenLoop(ClimberConstants.CLIMB_EXTEND_VOLTAGE)

        if (climber.isAtTargetedPosition || currentRequest is Request.SuperstructureRequest.Idle) {
          currentState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.CLIMB_RETRACT -> {
        climber.currentRequest =
          Request.ClimberRequest.OpenLoop(ClimberConstants.CLIMB_RETRACT_VOLTAGE)

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          currentState = SuperstructureStates.IDLE
        }
      }
    }

    if (nextState != currentState) {
      lastTransitionTime = Clock.fpgaTime
    }

    lastCoralScoringLevel = coralScoringLevel
    lastAlgaeIntakeLevel = algaeIntakeLevel
    currentState = nextState
  }

  // General Commands
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

  fun ejectCommand(): Command {
    val returnCommand = run {
      currentRequest = Request.SuperstructureRequest.Eject()
      rollers.currentRequest = Request.RollersRequest.OpenLoop(8.0.volts)
      ramp.currentRequest = Request.RampRequest.OpenLoop(5.0.volts)
    }
    returnCommand.name = "EjectCommand"
    return returnCommand
  }

  fun manualResetCommand(): Command {
    val returnCommand = run {
      currentRequest = Request.SuperstructureRequest.ManualReset()
    }

    returnCommand.name = "ManualResetCommand"
    return returnCommand
  }

  // Coral Commands
  fun intakeCoralCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.IntakeCoral() }.until {
        isAtRequestedState && currentState == SuperstructureStates.PREP_INTAKE_CORAL
      }
    returnCommand.name = "IntakeCoralCommand"
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

  // Algae Commands
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

  // Score Commands
  fun scoreCommand(): Command {
    val returnCommand = runOnce { currentRequest = Request.SuperstructureRequest.Score() }
    returnCommand.name = "ScoreCommand"
    return returnCommand
  }

  fun prepScoreDefaultCommand(): Command {
    val returnCommand =
      run {
        if (currentState == SuperstructureStates.IDLE) {
          when (theoreticalGamePiece) {
            GamePiece.CORAL ->
              currentRequest = Request.SuperstructureRequest.ScorePrepCoral(CoralLevel.L1)
            GamePiece.ALGAE ->
              currentRequest = Request.SuperstructureRequest.ScorePrepAlgaeProcessor()
          }
        } else if (currentState == SuperstructureStates.PREP_SCORE_CORAL &&
          coralScoringLevel != CoralLevel.L1
        ) {
          currentRequest = Request.SuperstructureRequest.Score()
        }
      }
        .until({
          currentState == SuperstructureStates.PREP_SCORE_CORAL ||
            currentState == SuperstructureStates.PREP_SCORE_ALGAE_PROCESSOR
        })

    returnCommand.name = "prepScoreDefaultCommand"
    return returnCommand
  }

  // Climber Commands
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

  companion object {
    enum class SuperstructureStates {
      UNINITIALIZED,
      TUNING,
      MANUAL_RESET,
      EJECT,
      HOME_PREP,
      HOME,
      IDLE,
      PREP_INTAKE_CORAL,
      INTAKE_CORAL,
      PREP_SCORE_CORAL,
      SCORE_CORAL,
      PREP_INTAKE_ALGAE,
      INTAKE_ALGAE,
      PREP_SCORE_ALGAE_PROCESSOR,
      SCORE_ALGAE_PROCESSOR,
      PREP_SCORE_ALGAE_BARGE,
      SCORE_ALGAE_BARGE,
      CLIMB_EXTEND,
      CLIMB_RETRACT,
    }
  }
}
