package com.team4099.robot2025.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.subsystems.led.Leds
import com.team4099.robot2025.config.constants.Constants.Universal.AlgaeLevel
import com.team4099.robot2025.config.constants.Constants.Universal.CoralLevel
import com.team4099.robot2025.config.constants.Constants.Universal.GamePiece
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.subsystems.arm.Arm
import com.team4099.robot2025.subsystems.climber.Climber
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorTunableValues
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.rollers.Ramp
import com.team4099.robot2025.subsystems.rollers.RampTunableValues
import com.team4099.robot2025.subsystems.rollers.Rollers
import com.team4099.robot2025.subsystems.superstructure.Superstructure.Companion.SuperstructureStates
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.milli

class Superstructure2 (
    private val elevator: Elevator,
    private val rollers: Rollers,
    private val ramp: Ramp,
    private val arm: Arm,
    private val climber: Climber,
    private val leds: Leds,
    private val vision: Vision,
    ) : SubsystemBase() {
    var theoreticalGamePiece: GamePiece = GamePiece.NONE
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

    private var cleaningUpIntake: Boolean = false

    override fun periodic() {
        // led updates
        leds.hasCoral =
            theoreticalGamePiece == GamePiece.CORAL
        leds.isAutoAligning = vision.isAutoAligning
        leds.isAligned = vision.isAligned
        leds.seesTag = Clock.fpgaTime - vision.lastTrigVisionUpdate.timestamp < 500.milli.seconds
        leds.isIntaking =
            currentState == SuperstructureStates.INTAKE_CORAL ||
                    currentState == SuperstructureStates.PREP_INTAKE_CORAL ||
                    currentState == SuperstructureStates.INTAKE_CORAL_CLEANUP ||
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
        val armAngle = arm.inputs.armPosition

        // First Elevator Stage
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

        // Second Elevator Stage
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
            // General States
            SuperstructureStates.UNINITIALIZED -> {
                nextState = SuperstructureStates.HOME_PREP
            }
            SuperstructureStates.TUNING -> {
                if(currentRequest is Request.SuperstructureRequest.Idle) {
                    nextState = SuperstructureStates.IDLE
                }
            }
            SuperstructureStates.MANUAL_RESET -> {

            }
            SuperstructureStates.EJECT -> {

            }
            SuperstructureStates.HOME_PREP -> {
                arm.currentRequest = Request.ArmRequest.Zero()

                if(arm.isZeroed) {
                    nextState = SuperstructureStates.HOME
                }

            }
            SuperstructureStates.HOME -> {
                elevator.currentRequest = Request.ElevatorRequest.Home()
            }
            SuperstructureStates.IDLE -> {
                ramp.currentRequest = Request.RampRequest.OpenLoop(RampTunableValues.idleVoltage.get())
            }

            // Coral States
            SuperstructureStates.PREP_INTAKE_CORAL -> {
                elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.ElevatorHeights.intak  q eCoralHeight.get())
                arm.currentRequest = Reu
            }
            SuperstructureStates.INTAKE_CORAL -> {

            }
            SuperstructureStates.INTAKE_CORAL_CLEANUP ->  {

            }
            SuperstructureStates.PREP_SCORE_CORAL ->  {

            }
            SuperstructureStates.SCORE_CORAL -> {

            }

            // Algae States
            SuperstructureStates.PREP_INTAKE_ALGAE -> {

            }
            SuperstructureStates.INTAKE_ALGAE -> {

            }
            SuperstructureStates.PREP_SCORE_ALGAE_PROCESSOR -> {

            }
            SuperstructureStates.SCORE_ALGAE_PROCESSOR -> {

            }
            SuperstructureStates.PREP_SCORE_ALGAE_BARGE -> {

            }
            SuperstructureStates.SCORE_ALGAE_BARGE -> {

            }

            // General Scoring States
            SuperstructureStates.SCORE_ACTION_CLEANUP -> {

            }

            // Climber States
            SuperstructureStates.CLIMB_EXTEND -> {

            }
            SuperstructureStates.CLIMB_RETRACT -> {

            }
        }
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
            INTAKE_CORAL_CLEANUP,
            PREP_SCORE_CORAL,
            SCORE_CORAL,

            PREP_INTAKE_ALGAE,
            INTAKE_ALGAE,
            PREP_SCORE_ALGAE_PROCESSOR,
            SCORE_ALGAE_PROCESSOR,
            PREP_SCORE_ALGAE_BARGE,
            SCORE_ALGAE_BARGE,

            SCORE_ACTION_CLEANUP,

            CLIMB_EXTEND,
            CLIMB_RETRACT,
        }
    }
}