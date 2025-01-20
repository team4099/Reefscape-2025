package com.team4099.robot2025.subsystems.climber

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.volts

class Climber(private val io: ClimberIO) {
  val inputs: ClimberIO.ClimberInputs = ClimberIO.ClimberInputs()
  private var currentState: ClimberState = ClimberState.UNINITIALIZED
  private var feedforward: ArmFeedforward

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
    get() =
      currentState == ClimberState.CLOSED_LOOP &&
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
      ClimberTunableValues.kPSlot0.initDefault(ClimberConstants.PID.KP_UNLATCH)
      ClimberTunableValues.kISlot0.initDefault(ClimberConstants.PID.KI_UNLATCH)
      ClimberTunableValues.kDSlot0.initDefault(ClimberConstants.PID.KD_UNLATCH)

      ClimberTunableValues.kPSlot1.initDefault(ClimberConstants.PID.KP_LATCH)
      ClimberTunableValues.kISlot1.initDefault(ClimberConstants.PID.KI_LATCH)
      ClimberTunableValues.kDSlot1.initDefault(ClimberConstants.PID.KD_LATCH)

      ClimberTunableValues.kS.initDefault(ClimberConstants.PID.KS_REAL)
      ClimberTunableValues.kG.initDefault(ClimberConstants.PID.KG_REAL)
      ClimberTunableValues.kV.initDefault(ClimberConstants.PID.KV_REAL)
      ClimberTunableValues.kA.initDefault(ClimberConstants.PID.KA_REAL)

      feedforward =
        ArmFeedforward(
          ClimberConstants.PID.KS_REAL,
          ClimberConstants.PID.KG_REAL,
          ClimberConstants.PID.KV_REAL,
          ClimberConstants.PID.KA_REAL
        )
    } else {
      ClimberTunableValues.kPSlot0.initDefault(ClimberConstants.PID.KP_SIM)
      ClimberTunableValues.kISlot0.initDefault(ClimberConstants.PID.KI_SIM)
      ClimberTunableValues.kDSlot0.initDefault(ClimberConstants.PID.KD_SIM)

      ClimberTunableValues.kG.initDefault(ClimberConstants.PID.KG_SIM)
      ClimberTunableValues.kV.initDefault(ClimberConstants.PID.KV_SIM)
      ClimberTunableValues.kA.initDefault(ClimberConstants.PID.KA_SIM)

      // ClimberTunableValues.kS is 0 volts because no static friction in sim
      feedforward =
        ArmFeedforward(
          0.volts,
          ClimberConstants.PID.KG_SIM,
          ClimberConstants.PID.KV_SIM,
          ClimberConstants.PID.KA_SIM
        )
    }
  }

  fun periodic() {
    io.updateInputs(inputs)

    if (ClimberTunableValues.kPSlot0.hasChanged() ||
      ClimberTunableValues.kISlot0.hasChanged() ||
      ClimberTunableValues.kDSlot0.hasChanged()
    ) {
      io.configPIDSlot0(
        ClimberTunableValues.kPSlot0.get(),
        ClimberTunableValues.kISlot0.get(),
        ClimberTunableValues.kDSlot0.get()
      )
    }

    if (ClimberTunableValues.kPSlot1.hasChanged() ||
      ClimberTunableValues.kISlot1.hasChanged() ||
      ClimberTunableValues.kDSlot1.hasChanged()
    ) {
      io.configPIDSlot0(
        ClimberTunableValues.kPSlot1.get(),
        ClimberTunableValues.kISlot1.get(),
        ClimberTunableValues.kDSlot1.get()
      )
    }

    if (ClimberTunableValues.kS.hasChanged() ||
      ClimberTunableValues.kG.hasChanged() ||
      ClimberTunableValues.kV.hasChanged() ||
      ClimberTunableValues.kA.hasChanged()
    ) {
      feedforward =
        ArmFeedforward(
          ClimberTunableValues.kS.get(),
          ClimberTunableValues.kG.get(),
          ClimberTunableValues.kV.get(),
          ClimberTunableValues.kA.get()
        )
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
          (Clock.fpgaTime - lastHomingCurrentSpikeTime) <
          ClimberConstants.HOMING_STALL_TIME_THRESHOLD
        ) {
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
    return (maxAngleReached && voltage > 0.volts) || (minAngleReached && voltage < 0.volts)
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
