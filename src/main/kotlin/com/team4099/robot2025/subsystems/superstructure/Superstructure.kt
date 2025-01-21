package com.team4099.robot2025.subsystems.superstructure

import com.team4099.robot2025.config.constants.Constants.Universal.AlgaeLevel
import com.team4099.robot2025.config.constants.Constants.Universal.CoralLevel
import com.team4099.robot2025.config.constants.Constants.Universal.GamePiece
import com.team4099.robot2025.subsystems.arm.Arm
import com.team4099.robot2025.subsystems.arm.ArmTunableValues
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorTunableValues
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.rollers.Rollers
import com.team4099.robot2025.subsystems.rollers.RollersTunableValues
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Superstructure(
  private val drivetrain: Drivetrain,
  private val elevator: Elevator,
  private val rollers: Rollers,
  private val arm: Arm,
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
            is Request.SuperstructureRequest.IntakeCoral -> SuperstructureStates.PREP_INTAKE_CORAL
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
      SuperstructureStates.PREP_INTAKE_CORAL -> {
        elevator.currentRequest = Request.ElevatorRequest.Home()
        if (elevator.isHomed) {
          arm.currentRequest =
            Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.intakeCoralAngle.get())
        }
      }
      SuperstructureStates.INTAKE_CORAL -> {
        rollers.currentRequest =
          Request.RollersRequest.OpenLoop(RollersTunableValues.intakeVoltage.get())
      }
      SuperstructureStates.CLEANUP_INTAKE_CORAL -> {
        /* flip to opposite intake side */
        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(ArmTunableValues.ArmAngles.idleCoralAngle.get())
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
          Request.RollersRequest.OpenLoop(RollersTunableValues.intakeVoltage.get())
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
          Request.RollersRequest.OpenLoop(RollersTunableValues.intakeVoltage.get())

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
          Request.RollersRequest.OpenLoop(RollersTunableValues.scoreVoltage.get())
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
          Request.RollersRequest.OpenLoop(RollersTunableValues.scoreVoltage.get())
      }
      SuperstructureStates.CLIMB_EXTEND -> {}
      SuperstructureStates.CLIMB_RETRACT -> {}
    }

    currentState = nextState
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
      CLEANUP_INTAKE_CORAL,
      PREP_INTAKE_ALGAE,
      INTAKE_ALGAE,
      PREP_SCORE_ALGAE_PROCESSOR,
      SCORE_ALGAE_PROCESSOR,
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
