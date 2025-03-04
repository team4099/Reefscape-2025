package com.team4099.robot2025.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.subsystems.led.Leds
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants.Universal.AlgaeLevel
import com.team4099.robot2025.config.constants.Constants.Universal.CoralLevel
import com.team4099.robot2025.config.constants.Constants.Universal.GamePiece
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.config.constants.RollersConstants
import com.team4099.robot2025.subsystems.arm.Arm
import com.team4099.robot2025.subsystems.arm.ArmTunableValues
import com.team4099.robot2025.subsystems.climber.Climber
import com.team4099.robot2025.subsystems.climber.ClimberTunableValues
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
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.sin
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

  private var theoreticalGamePiece: GamePiece = GamePiece.NONE
    get() {
      if (rollers.hasCoralVertical || ramp.hasCoral) {
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

  private var readyForCleanup: Boolean = true
  private var lastCleanupTime = Clock.fpgaTime

  override fun periodic() {

    // led updates
    leds.hasCoral =
      theoreticalGamePiece == GamePiece.CORAL || theoreticalGamePiece == GamePiece.CORAL_L1
    var isAutoAligning = vision.isAutoAligning
    var isAligned = vision.isAligned
    var seesTag = Clock.fpgaTime - vision.lastTrigVisionUpdate.timestamp < 100.milli.seconds

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
    val armAngle = arm.inputs.armPosition
    // Firsts Stage
    CustomLogger.recordOutput(
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

    // Second Stage
    CustomLogger.recordOutput(
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
    CustomLogger.recordOutput(
      "SimulatedMechanisms/2",
      Pose3d()
        .transformBy(
          Transform3d(Translation3d(0.0.inches, 0.0.inches, elevatorPosition), Rotation3d())
        )
        .pose3d
    )

    // Arm
    CustomLogger.recordOutput(
      "SimulatedMechanisms/3",
      Pose3d()
        .transformBy(
          Transform3d(
            Translation3d(0.21.meters, 0.meters, 0.38.meters + elevatorPosition),
            Rotation3d(0.degrees, -armAngle, 0.degrees)
          )
        )
        .pose3d
    )

    // Manipulator
    CustomLogger.recordOutput(
      "SimulatedMechanisms/4",
      Pose3d()
        .transformBy(
          Transform3d(
            Translation3d(
              0.21.meters + 0.2775.meters * armAngle.cos,
              0.meters,
              0.38.meters + 0.2775.meters * armAngle.sin + elevatorPosition
            ),
            Rotation3d()
          )
        )
        .pose3d
    )

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
          elevator.currentRequest = Request.ElevatorRequest.OpenLoop(0.0.volts)
          nextState = SuperstructureStates.TUNING
        }
      }
      SuperstructureStates.TUNING -> {

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.IDLE -> {
        climber.currentRequest =
          Request.ClimberRequest.ClosedLoop(ClimberTunableValues.climbIdleAngle.get())
        ramp.currentRequest = Request.RampRequest.OpenLoop(RampTunableValues.idleVoltage.get())

        // mechanism idle positions based on which game piece robot has
        when (theoreticalGamePiece) {
          GamePiece.CORAL -> {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.idleCoralAngle.get())

            if ((Clock.fpgaTime - lastTransitionTime) >= 0.5.seconds && (Clock.fpgaTime - lastTransitionTime) <= 0.7.seconds && !DriverStation.isAutonomous()) {
              rollers.currentRequest =
                Request.RollersRequest.OpenLoop(RollersConstants.CLEANUP_CORAL_VOLTAGE)
            }
            else {
              rollers.currentRequest =
                Request.RollersRequest.OpenLoop(RollersTunableValues.idleCoralVoltage.get())
            }

            if (arm.isAtTargetedPosition) {
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  ElevatorTunableValues.ElevatorHeights.idleCoralVerticalHeight.get()
                )
            }
          }
          GamePiece.CORAL_L1 -> {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.idleCoralL1Angle.get())
            rollers.currentRequest =
                Request.RollersRequest.OpenLoop(RollersTunableValues.idleCoralL1Voltage.get())

            if (arm.isAtTargetedPosition) {
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  ElevatorTunableValues.ElevatorHeights.idleCoralHorizontalHeight.get()
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
              Request.RollersRequest.OpenLoop(RampTunableValues.idleVoltage.get())

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
              SuperstructureStates.PREP_ARM_PASS_THROUGH
            is Request.SuperstructureRequest.IntakeL1 ->
              SuperstructureStates.PREP_ELEVATOR_MOVEMENT
            is Request.SuperstructureRequest.ScorePrepCoral ->
              SuperstructureStates.PREP_ELEVATOR_MOVEMENT
            is Request.SuperstructureRequest.ScorePrepAlgaeProcessor ->
              SuperstructureStates.PREP_ELEVATOR_MOVEMENT
            is Request.SuperstructureRequest.ScorePrepAlgaeBarge ->
              SuperstructureStates.PREP_ELEVATOR_MOVEMENT
            is Request.SuperstructureRequest.ClimbExtend -> SuperstructureStates.CLIMB_EXTEND
            is Request.SuperstructureRequest.ClimbRetract -> SuperstructureStates.CLIMB_RETRACT
            is Request.SuperstructureRequest.Tuning -> SuperstructureStates.TUNING
            else -> currentState
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
            is Request.SuperstructureRequest.IntakeCoral -> {
              if (arm.inputs.armPosition < ArmTunableValues.ArmAngles.safeElevatorBackAngle.get()) {
                nextState = SuperstructureStates.PREP_INTAKE_CORAL
              }
            }
            is Request.SuperstructureRequest.IntakeL1 -> {
              nextState = SuperstructureStates.PREP_INTAKE_L1
            }
            is Request.SuperstructureRequest.IntakeAlgae -> {
              nextState = SuperstructureStates.PREP_INTAKE_ALGAE
            }
            is Request.SuperstructureRequest.ScorePrepAlgaeProcessor -> {
              nextState = SuperstructureStates.PREP_SCORE_ALGAE_PROCESSOR
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
      SuperstructureStates.PREP_ARM_PASS_THROUGH -> {
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(
            ElevatorTunableValues.ElevatorHeights.passThroughHeight.get()
          )
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.idleCoralVoltage.get())
        ramp.currentRequest = Request.RampRequest.OpenLoop(RampTunableValues.idleVoltage.get())

        if (elevator.isAtTargetedPosition) {
          when (currentRequest) {
            is Request.SuperstructureRequest.IntakeCoral -> {
              nextState = SuperstructureStates.PREP_INTAKE_CORAL
            }
            is Request.SuperstructureRequest.Idle -> {
              nextState = SuperstructureStates.IDLE
            }
          }
        }
      }
      SuperstructureStates.PREP_INTAKE_CORAL -> {
        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.intakeCoralAngle.get())

        if (arm.isAtTargetedPosition) {
          nextState = SuperstructureStates.INTAKE_CORAL
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.INTAKE_CORAL -> {
        if (!rollers.hasCoralVertical) {
          ramp.currentRequest =
            Request.RampRequest.OpenLoop(RampTunableValues.intakeCoralVoltageFast.get())
          rollers.currentRequest =
            Request.RollersRequest.OpenLoop(RollersTunableValues.intakeCoralVoltageFast.get())
        } else {
          currentRequest = Request.SuperstructureRequest.Idle()
          theoreticalGamePiece = GamePiece.CORAL
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.PREP_ARM_PASS_THROUGH
        }
      }
      SuperstructureStates.PREP_INTAKE_L1 -> {
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(
            ElevatorTunableValues.ElevatorHeights.intakeL1Height.get()
          )
        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.intakeL1Angle.get())
        if (arm.isAtTargetedPosition) {
          nextState = SuperstructureStates.INTAKE_L1
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
        }
      }
      SuperstructureStates.INTAKE_L1 -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.intakeL1Voltage.get())

        if (rollers.hasCoralHorizontal) {
          theoreticalGamePiece = GamePiece.CORAL_L1
          nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
        }
      }
      SuperstructureStates.PREP_SCORE_CORAL -> {
        if (coralScoringLevel != lastCoralScoringLevel) {
          nextState = SuperstructureStates.PREP_ELEVATOR_MOVEMENT
        } else {
          val reefLevelElevatorHeight: Length =
            when (coralScoringLevel) {
              CoralLevel.L1 -> {
                if (theoreticalGamePiece == GamePiece.CORAL)
                  ElevatorTunableValues.ElevatorHeights.L1VerticalHeight.get()
                else ElevatorTunableValues.ElevatorHeights.L1HorizontalHeight.get()
              }
              CoralLevel.L2 -> ElevatorTunableValues.ElevatorHeights.L2Height.get()
              CoralLevel.L3 -> ElevatorTunableValues.ElevatorHeights.L3Height.get()
              CoralLevel.L4 -> ElevatorTunableValues.ElevatorHeights.L4Height.get()
              else -> 0.0.inches
            }

          val reefLevelArmAngle: Angle =
            when (coralScoringLevel) {
              CoralLevel.L1 -> {
                if (theoreticalGamePiece == GamePiece.CORAL)
                  ArmTunableValues.ArmAngles.scoreCoralL1VerticalAngle.get()
                else ArmTunableValues.ArmAngles.scoreCoralL1HorizontalAngle.get()
              }
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

        when (currentRequest) {
          is Request.SuperstructureRequest.Score -> {
            if (arm.isAtTargetedPosition && elevator.isAtTargetedPosition) {
              nextState = SuperstructureStates.SCORE_CORAL
            }
          }
          is Request.SuperstructureRequest.IntakeAlgae -> {
            nextState = SuperstructureStates.PREP_ELEVATOR_MOVEMENT
          }
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
          }
        }
      }
      SuperstructureStates.SCORE_CORAL -> {
        val reefLevelElevatorHeight: Length =
          when (coralScoringLevel) {
            CoralLevel.L1 -> {
              if (theoreticalGamePiece == GamePiece.CORAL)
                ElevatorTunableValues.ElevatorHeights.L1VerticalHeight.get()
              else ElevatorTunableValues.ElevatorHeights.L1HorizontalHeight.get()
            }
            CoralLevel.L2 -> ElevatorTunableValues.ElevatorHeights.L2Height.get()
            CoralLevel.L3 -> ElevatorTunableValues.ElevatorHeights.L3Height.get()
            CoralLevel.L4 -> ElevatorTunableValues.ElevatorHeights.L4Height.get()
            else -> 0.0.inches
          }

        val reefLevelArmAngle: Angle =
          when (coralScoringLevel) {
            CoralLevel.L1 -> {
              if (theoreticalGamePiece == GamePiece.CORAL)
                ArmTunableValues.ArmAngles.scoreCoralL1VerticalAngle.get()
              else ArmTunableValues.ArmAngles.scoreCoralL1HorizontalAngle.get()
            }
            CoralLevel.L2 -> ArmTunableValues.ArmAngles.scoreCoralL2Angle.get()
            CoralLevel.L3 -> ArmTunableValues.ArmAngles.scoreCoralL3Angle.get()
            CoralLevel.L4 -> ArmTunableValues.ArmAngles.scoreCoralL4Angle.get()
            else -> {
              0.0.degrees
            }
          }

        if (theoreticalGamePiece == GamePiece.CORAL || theoreticalGamePiece == GamePiece.NONE) {
//          if (coralScoringLevel == CoralLevel.L4) {
//            elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(reefLevelElevatorHeight)
//          }

//          if (elevator.isAtTargetedPosition
//            && elevator.elevatorPositionTarget == ElevatorTunableValues.ElevatorHeights.L4Height.get()) {
//            elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(reefLevelElevatorHeight + 3.inches)
//            rollers.currentRequest = Request.RollersRequest.OpenLoop(RollersTunableValues.scoreCoralVoltage.get())
//          }

          if (coralScoringLevel != CoralLevel.L1 && elevator.isAtTargetedPosition) {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                reefLevelArmAngle +
                  (
                    if (coralScoringLevel == CoralLevel.L4) ArmConstants.SCORE_OFFSET // 0.degrees
                    else ArmConstants.SCORE_OFFSET
                    )
              )

//            if (coralScoringLevel == CoralLevel.L4 && elevator.isAtTargetedPosition) {
//              elevator.currentRequest =
//                Request.ElevatorRequest.ClosedLoop(reefLevelElevatorHeight + 3.inches)
//            }
          }

          val isReadyToScore =
            if (coralScoringLevel == CoralLevel.L4)
              elevator.isAtTargetedPosition && arm.isAtTargetedPosition
            else arm.isAtTargetedPosition

          if (isReadyToScore) {
            rollers.currentRequest =
              Request.RollersRequest.OpenLoop(RollersTunableValues.scoreCoralVoltage.get())
          }

          val spitOutTime =
            if (coralScoringLevel == CoralLevel.L4)
              (RollersTunableValues.coralSpitTime.get() + 0.4.seconds) // 0.5.seconds
            else RollersTunableValues.coralSpitTime.get()

          if ((Clock.fpgaTime - lastTransitionTime) > spitOutTime) {
            theoreticalGamePiece = GamePiece.NONE
            nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
          }
        } else if (theoreticalGamePiece == GamePiece.CORAL_L1) {
          rollers.currentRequest =
            Request.RollersRequest.OpenLoop(RollersTunableValues.scoreCoralL1Voltage.get())
          if ((Clock.fpgaTime - lastTransitionTime) > RollersTunableValues.coralL1SpitTime.get()) {
            theoreticalGamePiece = GamePiece.NONE
            nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
          }
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
        }
      }
      SuperstructureStates.FRONT_ACTION_CLEANUP -> {
        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.safeElevatorFrontAngle.get())

        if (arm.isAtTargetedPosition) {
          val elevatorIdleHeight =
            when (theoreticalGamePiece) {
              GamePiece.CORAL ->
                ElevatorTunableValues.ElevatorHeights.idleCoralHorizontalHeight.get()
              GamePiece.ALGAE -> ElevatorTunableValues.ElevatorHeights.idleAlgaeHeight.get()
              GamePiece.CORAL_L1 ->
                ElevatorTunableValues.ElevatorHeights.idleCoralVerticalHeight.get()
              GamePiece.NONE -> ElevatorTunableValues.ElevatorHeights.idleHeight.get()
            }

          elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(elevatorIdleHeight)

          if (elevator.isAtTargetedPosition) {
            currentRequest = Request.SuperstructureRequest.Idle()
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.PREP_INTAKE_ALGAE -> {
        when (currentRequest) {
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

              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(algaeIntakeElevatorHeight)

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
            nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
          }
        }
      }
      SuperstructureStates.INTAKE_ALGAE -> {

        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.intakeAlgaeVoltage.get())

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
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
          nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
        }
      }
      SuperstructureStates.SCORE_ALGAE_PROCESSOR -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.scoreProcessorAlgaeVoltage.get())

        if ((Clock.fpgaTime - lastTransitionTime) >
          RollersTunableValues.algaeProcessorSpitTime.get()
        ) {
          theoreticalGamePiece = GamePiece.NONE
          nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
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
          nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
        }
      }
      SuperstructureStates.SCORE_ALGAE_BARGE -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.scoreBargeAlgaeVoltage.get())

        if ((Clock.fpgaTime - lastTransitionTime) > RollersTunableValues.algaeBargeSpitTime.get()) {
          theoreticalGamePiece = GamePiece.NONE
          nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
        }

        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.FRONT_ACTION_CLEANUP
        }
      }
      SuperstructureStates.CLIMB_EXTEND -> {
        climber.currentRequest =
          Request.ClimberRequest.ClosedLoop(ClimberTunableValues.climbExtendAngle.get())

        if (climber.isAtTargetedPosition &&
          currentRequest is Request.SuperstructureRequest.ClimbRetract
        ) {
          currentState = SuperstructureStates.CLIMB_RETRACT
        }
        if (currentRequest is Request.SuperstructureRequest.Idle) {
          currentState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.CLIMB_RETRACT -> {
        climber.currentRequest =
          Request.ClimberRequest.ClosedLoop(ClimberTunableValues.climbRetractAngle.get())

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

  fun intakeL1Command(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.IntakeL1() }.until {
        isAtRequestedState && currentState == SuperstructureStates.PREP_INTAKE_L1
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
    val returnCommand = runOnce { currentRequest = Request.SuperstructureRequest.Score() }
    returnCommand.name = "ScoreCommand"
    return returnCommand
  }

  fun prepScoreDefaultCommand(): Command {
    val returnCommand =
      run {
        if (currentState == SuperstructureStates.IDLE ||
          currentState == SuperstructureStates.PREP_ELEVATOR_MOVEMENT
        ) {
          when (theoreticalGamePiece) {
            GamePiece.CORAL_L1 ->
              currentRequest = Request.SuperstructureRequest.ScorePrepCoral(CoralLevel.L1)
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

  fun testArmDownCommand(): Command {
    val returnCommand = run {
      currentRequest = Request.SuperstructureRequest.Tuning()
      arm.currentRequest = Request.ArmRequest.ClosedLoop(0.0.degrees)

      if (arm.isAtTargetedPosition) {
        currentRequest = Request.SuperstructureRequest.Idle()
      }
    }
    returnCommand.name = "TestArmDownCommand"
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
      elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(5.inches)

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

  fun ejectCommand(): Command {
    val returnCommand = run {
      currentRequest = Request.SuperstructureRequest.Eject()
      elevator.currentRequest = Request.ElevatorRequest.Home()
      arm.currentRequest = Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.intakeCoralAngle.get());
      rollers.currentRequest = Request.RollersRequest.OpenLoop(8.0.volts)
    }
    returnCommand.name = "EjectCommand"
    return returnCommand
  }

  companion object {
    enum class SuperstructureStates {
      UNINITIALIZED,
      TUNING,
      IDLE,
      HOME_PREP,
      HOME,
      PREP_ARM_PASS_THROUGH,
      PREP_INTAKE_CORAL,
      INTAKE_CORAL,
      INTAKE_CORAL_CLEANUP,
      PREP_INTAKE_L1,
      INTAKE_L1,
      PREP_INTAKE_ALGAE,
      INTAKE_ALGAE,
      PREP_SCORE_ALGAE_PROCESSOR,
      SCORE_ALGAE_PROCESSOR,
      PREP_ELEVATOR_MOVEMENT,
      PREP_SCORE_CORAL,
      SCORE_CORAL,
      FRONT_ACTION_CLEANUP,
      CLIMB_EXTEND,
      CLIMB_RETRACT,
      PREP_SCORE_ALGAE_BARGE,
      SCORE_ALGAE_BARGE,
      EJECT
    }
  }
}
