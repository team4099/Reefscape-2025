package com.team4099.robot2025.subsystems.climber

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond

class Climber(private val io: ClimberIO) {
    val inputs: ClimberIO.ClimberInputs = ClimberIO.ClimberInputs()
    private var currentState: ClimberStates = ClimberStates.UNINITIALIZED
    private var feedforward: SimpleMotorFeedforward<Radian, Volt>

    private val kS = LoggedTunableValue(
        "Climber/kS",
        Pair({it.inVolts}, {it.volts})
    )

    private val kV = LoggedTunableValue(
        "Climber/kV",
        Pair({it.inVoltsPerRadianPerSecond}, {it.volts.perRadianPerSecond})
    )

    private val kA = LoggedTunableValue(
        "Climber/kA",
        Pair({it.inVoltsPerRadianPerSecondPerSecond}, {it.volts.perRadianPerSecondPerSecond})
    )

    private val kP = LoggedTunableValue(
        "Climber/kP",
        Pair({ it.inVoltsPerRadian }, { it.volts.perRadian })
    )

    private val kI = LoggedTunableValue(
        "Climber/kI",
        Pair({ it.inVoltsPerRadianSeconds }, { it.volts.perRadianSeconds })
    )

    private val kD = LoggedTunableValue(
        "Climber/kD",
        Pair({ it.inVoltsPerRadianPerSecond }, { it.volts.perRadianPerSecond })
    )

    private val kPSlot1 = LoggedTunableValue(
        "Climber/kPSlot1",
        Pair({ it.inVoltsPerRadian }, { it.volts.perRadian })
    )

    private val kISlot1 = LoggedTunableValue(
        "Climber/kISlot1",
        Pair({ it.inVoltsPerRadianSeconds }, { it.volts.perRadianSeconds })
    )

    private val kDSlot1 = LoggedTunableValue(
        "Climber/kDSlot1",
        Pair({ it.inVoltsPerRadianPerSecond }, { it.volts.perRadianPerSecond })
    )

    private var climberConstraints: TrapezoidProfile.Constraints<Radian> = TrapezoidProfile.Constraints(
        ClimberConstants.MAX_VELOCITY,
        ClimberConstants.MAX_ACCELERATION
    )

    private var climberProfile = TrapezoidProfile(
        climberConstraints,
        TrapezoidProfile.State((-1337.0).radians, -1337.0.radians.perSecond),
        TrapezoidProfile.State((-1337.0).radians, -1337.0.radians.perSecond)
    )

    private val maxAngleReached: Boolean
        get() = inputs.climberPosition >= ClimberConstants.MAX_ANGLE

    private val minAngleReached: Boolean
        get() = inputs.climberPosition <= ClimberConstants.MIN_ANGLE

    private var targetPosition: Angle = 0.radians
    private var targetVoltage: ElectricalPotential = 0.volts
    private var lastTargetPosition: Angle = (-1337).radians
    private var lastVoltage = (-1337).volts
    private var lastSetpoint: TrapezoidProfile.State<Radian> = TrapezoidProfile.State()

    private var isHomed: Boolean = false
    private var lastHomingCurrentSpikeTime = Clock.fpgaTime
    private var climberTolerance: Angle = ClimberConstants.TOLERANCE
    private var timeProfileGeneratedAt = Clock.fpgaTime

    private val isAtTargetedPosition: Boolean
        get() =
            (currentState == ClimberStates.CLOSED_LOOP &&
                    climberProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
                    (inputs.climberPosition - targetPosition).absoluteValue <= climberTolerance) || inputs.isSimulated


    var currentRequest: Request.ClimberRequest = Request.ClimberRequest.Home()
        set(value) {
            when (value) {
                is Request.ClimberRequest.OpenLoop -> {
                    targetVoltage = value.voltage
                }

                is Request.ClimberRequest.ClosedLoop -> {
                    targetPosition = value.position
                }

                else -> {}
            }

            field = value
        }

    init {
        if (RobotBase.isReal()) {
            kP.initDefault(ClimberConstants.PID.KP_REAL)
            kI.initDefault(ClimberConstants.PID.KI_REAL)
            kD.initDefault(ClimberConstants.PID.KD_REAL)

            kPSlot1.initDefault(ClimberConstants.PID.KP_LATCH)
            kISlot1.initDefault(ClimberConstants.PID.KI_LATCH)
            kDSlot1.initDefault(ClimberConstants.PID.KD_LATCH)

            kS.initDefault(ClimberConstants.PID.KS)
            kV.initDefault(ClimberConstants.PID.KV)
            kA.initDefault(ClimberConstants.PID.KA)

            feedforward = SimpleMotorFeedforward(
                ClimberConstants.PID.KS,
                ClimberConstants.PID.KV,
                ClimberConstants.PID.KA
            )
        } else {
            kP.initDefault(ClimberConstants.PID.KP_SIM)
            kI.initDefault(ClimberConstants.PID.KI_SIM)
            kD.initDefault(ClimberConstants.PID.KD_SIM)

            kV.initDefault(ClimberConstants.PID.KV)
            kA.initDefault(ClimberConstants.PID.KA)

            // kS is 0 volts because no static friction in sim
            feedforward = SimpleMotorFeedforward(
                0.volts,
                ClimberConstants.PID.KV,
                ClimberConstants.PID.KA
            )
        }
    }

    fun periodic() {
        io.updateInputs(inputs)

        if(kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
            io.configPID(kP.get(), kI.get(), kD.get())
        }

        if(kPSlot1.hasChanged() || kISlot1.hasChanged() || kDSlot1.hasChanged()) {
            io.configPID(kPSlot1.get(), kISlot1.get(), kDSlot1.get())
        }

        if(kS.hasChanged() || kV.hasChanged() || kA.hasChanged()) {
            feedforward = SimpleMotorFeedforward(kS.get(), kV.get(), kA.get())
        }

        CustomLogger.processInputs("Climber", inputs)
        CustomLogger.recordOutput("Climber/currentState", currentState.name)
        CustomLogger.recordOutput("Climber/requestedState", currentRequest.javaClass.simpleName)
        CustomLogger.recordOutput("Climber/isAtTargetedPosition", isAtTargetedPosition)
        CustomLogger.recordOutput("Climber/requestedPosition", targetPosition.inRadians)

        var nextState = currentState

        when (currentState) {
            ClimberStates.UNINITIALIZED -> {
                nextState = fromRequestToState(currentRequest)
            }

            ClimberStates.OPEN_LOOP -> {
                setVoltage(targetVoltage)
                nextState = fromRequestToState(currentRequest)
            }

            ClimberStates.CLOSED_LOOP -> {
                if (targetPosition != lastTargetPosition) {
                    val preProfileGenerate = Clock.fpgaTime

                    climberProfile = TrapezoidProfile(
                        climberConstraints,
                        TrapezoidProfile.State(targetPosition, 0.radians.perSecond),
                        TrapezoidProfile.State(inputs.climberPosition, inputs.climberVelocity)
                    )

                    val postProfileGenerate = Clock.fpgaTime

                    CustomLogger.recordOutput(
                        "/Climber/ProfileGenerationMS",
                        postProfileGenerate.inSeconds - preProfileGenerate.inSeconds
                    )

                    timeProfileGeneratedAt = Clock.fpgaTime
                    lastTargetPosition = targetPosition
                }

                val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt
                setPosition(climberProfile.calculate(timeElapsed))
                nextState = fromRequestToState(currentRequest)

                if (!currentState.equivalentToRequest(currentRequest)) {
                    lastTargetPosition = (-1337).radians
                    lastVoltage = (-1337).volts
                }
            }

            ClimberStates.HOME -> {
                if (inputs.climberStatorCurrent < ClimberConstants.HOMING_STALL_CURRENT) {
                    lastHomingCurrentSpikeTime = Clock.fpgaTime
                }

                // Applies the homing voltage if not already homed and not stalled
                if (!isHomed &&
                    inputs.climberStatorCurrent < ClimberConstants.HOMING_STALL_CURRENT &&
                    (Clock.fpgaTime - lastHomingCurrentSpikeTime) < ClimberConstants.HOMING_STALL_TIME_THRESHOLD)
                {
                    setVoltage(ClimberConstants.HOMING_APPLIED_VOLTAGE)
                } else {
                    io.zeroEncoder()
                    currentRequest = Request.ClimberRequest.OpenLoop(0.volts)
                    isHomed = true
                    nextState = fromRequestToState(currentRequest)
                }
            }
        }

        currentState = nextState
    }

    private fun isOutOfBounds(setpointVelocity: AngularVelocity): Boolean {
        return (maxAngleReached && setpointVelocity > 0.radians.perSecond) ||
                (minAngleReached && setpointVelocity < 0.radians.perSecond)
    }

    private fun isOutOfBounds(voltage: ElectricalPotential): Boolean {
        return (maxAngleReached && voltage > 0.volts) ||
                (minAngleReached && voltage < 0.volts)
    }

    private fun setVoltage(voltage: ElectricalPotential) {
        if (isHomed && isOutOfBounds(voltage)) {
            io.setVoltage(0.volts)
        } else {
            io.setVoltage(voltage)
        }

        CustomLogger.recordDebugOutput("Climber/voltageOutOfBounds", isOutOfBounds(voltage))
    }

    private fun setPosition(setpoint: TrapezoidProfile.State<Radian>) {
        val acceleration = (setpoint.velocity - lastSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME
        lastSetpoint = setpoint

        val feedforward = feedforward.calculate(setpoint.velocity, acceleration)

        // TODO: Make set voltage for each motor
        if (isOutOfBounds(setpoint.velocity)) {
            io.setVoltage(0.volts)
        } else {
            io.setPosition(setpoint.position, feedforward)
        }

        CustomLogger.recordDebugOutput("Climber/profileOutOfBounds", isOutOfBounds(setpoint.velocity))
        CustomLogger.recordOutput("Climber/feedforward", feedforward.inVolts)
        CustomLogger.recordOutput("Climber/targetPosition", setpoint.position.inRadians)
        CustomLogger.recordOutput("Climber/targetVelocity", setpoint.velocity.inRadiansPerSecond)
    }

    companion object {
        enum class ClimberStates {
            UNINITIALIZED,
            OPEN_LOOP,
            CLOSED_LOOP,
            HOME;

            inline fun equivalentToRequest(request: Request.ClimberRequest): Boolean {
                return (
                        (request is Request.ClimberRequest.Home && this == HOME) ||
                                (request is Request.ClimberRequest.OpenLoop && this == OPEN_LOOP) ||
                                (request is Request.ClimberRequest.ClosedLoop && this == CLOSED_LOOP)
                        )
            }
        }

        inline fun fromRequestToState(request: Request.ClimberRequest): ClimberStates {
            return when (request) {
                is Request.ClimberRequest.Home -> ClimberStates.HOME
                is Request.ClimberRequest.OpenLoop -> ClimberStates.OPEN_LOOP
                is Request.ClimberRequest.ClosedLoop -> ClimberStates.CLOSED_LOOP
            }
        }
    }
}