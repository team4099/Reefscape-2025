package com.team4099.robot2025.subsystems.climber

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.units.derived.*

class Climber(private val io: ClimberIO) {
    val inputs: ClimberIO.ClimberInputs = ClimberIO.ClimberInputs()
    private var currentState: ClimberState = ClimberState.UNINITIALIZED
    private var feedforward: ArmFeedforward

    private val kS = LoggedTunableValue(
        "Climber/kS",
        Pair({it.inVolts}, {it.volts})
    )

    private val kV = LoggedTunableValue(
        "Climber/kV",
        Pair({it.inVoltsPerDegreePerSecond}, {it.volts.perDegreePerSecond})
    )

    private val kA = LoggedTunableValue(
        "Climber/kA",
        Pair({it.inVoltsPerDegreePerSecondPerSecond}, {it.volts.perDegreePerSecondPerSecond})
    )

    private val kG = LoggedTunableValue(
        "Climber/kG",
        Pair({it.inVolts}, {it.volts})
    )

    private val kPSlot0 = LoggedTunableValue(
        "Climber/kP",
        Pair({ it.inVoltsPerDegree }, { it.volts.perDegree })
    )

    private val kISlot0 = LoggedTunableValue(
        "Climber/kI",
        Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )

    private val kDSlot0 = LoggedTunableValue(
        "Climber/kD",
        Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

    private val kPSlot1 = LoggedTunableValue(
        "Climber/kPSlot1",
        Pair({ it.inVoltsPerDegree }, { it.volts.perDegree })
    )

    private val kISlot1 = LoggedTunableValue(
        "Climber/kISlot1",
        Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )

    private val kDSlot1 = LoggedTunableValue(
        "Climber/kDSlot1",
        Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

    private val maxAngleReached: Boolean
        get() = inputs.climberPosition >= ClimberConstants.MAX_ANGLE

    private val minAngleReached: Boolean
        get() = inputs.climberPosition <= ClimberConstants.MIN_ANGLE

    private var targetPosition: Angle = 0.degrees
    private var targetVoltage: ElectricalPotential = 0.volts
    private var lastTargetPosition: Angle = (-1337).degrees
    private var lastVoltage = (-1337).volts

    private var isHomed: Boolean = false
    private var lastHomingCurrentSpikeTime = Clock.fpgaTime
    private var climberTolerance: Angle = ClimberConstants.TOLERANCE
    private var latched: Boolean = false

    private val isAtTargetedPosition: Boolean
        get() = currentState == ClimberState.CLOSED_LOOP &&
                (inputs.climberPosition - targetPosition).absoluteValue <= climberTolerance ||
                inputs.isSimulated


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
            kPSlot0.initDefault(ClimberConstants.PID.KP_UNLATCH)
            kISlot0.initDefault(ClimberConstants.PID.KI_UNLATCH)
            kDSlot0.initDefault(ClimberConstants.PID.KD_UNLATCH)

            kPSlot1.initDefault(ClimberConstants.PID.KP_LATCH)
            kISlot1.initDefault(ClimberConstants.PID.KI_LATCH)
            kDSlot1.initDefault(ClimberConstants.PID.KD_LATCH)

            kS.initDefault(ClimberConstants.PID.KS_REAL)
            kG.initDefault(ClimberConstants.PID.KG_REAL)
            kV.initDefault(ClimberConstants.PID.KV_REAL)
            kA.initDefault(ClimberConstants.PID.KA_REAL)

            feedforward = ArmFeedforward(
                ClimberConstants.PID.KS_REAL,
                ClimberConstants.PID.KG_REAL,
                ClimberConstants.PID.KV_REAL,
                ClimberConstants.PID.KA_REAL
            )
        } else {
            kPSlot0.initDefault(ClimberConstants.PID.KP_SIM)
            kISlot0.initDefault(ClimberConstants.PID.KI_SIM)
            kDSlot0.initDefault(ClimberConstants.PID.KD_SIM)

            kG.initDefault(ClimberConstants.PID.KG_SIM)
            kV.initDefault(ClimberConstants.PID.KV_SIM)
            kA.initDefault(ClimberConstants.PID.KA_SIM)

            // kS is 0 volts because no static friction in sim
            feedforward = ArmFeedforward(
                0.volts,
                ClimberConstants.PID.KG_SIM,
                ClimberConstants.PID.KV_SIM,
                ClimberConstants.PID.KA_SIM
            )
        }
    }

    fun periodic() {
        io.updateInputs(inputs)

        if(kPSlot0.hasChanged() || kISlot0.hasChanged() || kDSlot0.hasChanged()) {
            io.configPIDSlot0(kPSlot0.get(), kISlot0.get(), kDSlot0.get())
        }

        if(kPSlot1.hasChanged() || kISlot1.hasChanged() || kDSlot1.hasChanged()) {
            io.configPIDSlot0(kPSlot1.get(), kISlot1.get(), kDSlot1.get())
        }

        if(kS.hasChanged() || kG.hasChanged() || kV.hasChanged() || kA.hasChanged()) {
            feedforward = ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get())
        }

        CustomLogger.processInputs("Climber", inputs)
        CustomLogger.recordOutput("Climber/currentState", currentState.name)
        CustomLogger.recordOutput("Climber/requestedState", currentRequest.javaClass.simpleName)
        CustomLogger.recordOutput("Climber/isAtTargetedPosition", isAtTargetedPosition)
        CustomLogger.recordOutput("Climber/requestedPosition", targetPosition.inDegrees)

        var nextState = currentState

        when (currentState) {
            ClimberState.UNINITIALIZED -> {
                nextState = fromRequestToState(currentRequest)
            }

            ClimberState.OPEN_LOOP -> {
                setVoltage(targetVoltage)
                nextState = fromRequestToState(currentRequest)
            }

            ClimberState.CLOSED_LOOP -> {
                if (targetPosition != lastTargetPosition) {
                    lastTargetPosition = targetPosition
                }

                io.setPosition(targetPosition, latched)
                nextState = fromRequestToState(currentRequest)

                if (!currentState.equivalentToRequest(currentRequest)) {
                    lastTargetPosition = (-1337).degrees
                    lastVoltage = (-1337).volts
                }
            }

            ClimberState.HOME -> {
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
    }

    companion object {
        enum class ClimberState {
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

        inline fun fromRequestToState(request: Request.ClimberRequest): ClimberState {
            return when (request) {
                is Request.ClimberRequest.Home -> ClimberState.HOME
                is Request.ClimberRequest.OpenLoop -> ClimberState.OPEN_LOOP
                is Request.ClimberRequest.ClosedLoop -> ClimberState.CLOSED_LOOP
            }
        }
    }
}