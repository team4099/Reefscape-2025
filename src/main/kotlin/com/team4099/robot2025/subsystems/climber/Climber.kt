package com.team4099.robot2025.subsystems.climber

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Climber(private val io: ClimberIO) {
  val inputs: ClimberIO.ClimberInputs = ClimberIO.ClimberInputs()
  private var currentState: ClimberState = ClimberState.UNINITIALIZED

  private val maxAngleReached: Boolean
    get() = inputs.climberPosition >= ClimberConstants.MAX_ANGLE

  private val minAngleReached: Boolean
    get() = inputs.climberPosition <= ClimberConstants.MIN_ANGLE

  private var targetPosition: Angle = 0.degrees
  private var targetVoltage: ElectricalPotential = 0.volts

  private var isHomed: Boolean = false
  private var lastHomingCurrentSpikeTime = Clock.fpgaTime
  private var climberTolerance: Angle = ClimberConstants.TOLERANCE

  val isAtTargetedPosition: Boolean
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
      ClimberTunableValues.kP.initDefault(ClimberConstants.PID.KP_REAL)
      ClimberTunableValues.kI.initDefault(ClimberConstants.PID.KI_REAL)
      ClimberTunableValues.kD.initDefault(ClimberConstants.PID.KD_REAL)
    } else {
      ClimberTunableValues.kP.initDefault(ClimberConstants.PID.KP_SIM)
      ClimberTunableValues.kI.initDefault(ClimberConstants.PID.KI_SIM)
      ClimberTunableValues.kD.initDefault(ClimberConstants.PID.KD_SIM)
    }
  }

  fun periodic() {
    io.updateInputs(inputs)

    if (ClimberTunableValues.kP.hasChanged() ||
      ClimberTunableValues.kI.hasChanged() ||
      ClimberTunableValues.kD.hasChanged()
    ) {
      io.configPID(
        ClimberTunableValues.kP.get(),
        ClimberTunableValues.kI.get(),
        ClimberTunableValues.kD.get()
      )
    }

    if (ClimberTunableValues.kS.hasChanged() ||
      ClimberTunableValues.kGDefault.hasChanged() ||
      ClimberTunableValues.kV.hasChanged() ||
      ClimberTunableValues.kA.hasChanged()
    ) {
      io.configFF(
        ClimberTunableValues.kGDefault.get(),
        ClimberTunableValues.kS.get(),
        ClimberTunableValues.kV.get(),
        ClimberTunableValues.kA.get()
      )
    }

    CustomLogger.processInputs("Climber", inputs)
    CustomLogger.recordOutput("Climber/currentState", currentState.name)
    CustomLogger.recordOutput("Climber/requestedState", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Climber/isAtTargetedPosition", isAtTargetedPosition)
    CustomLogger.recordOutput("Climber/targetPosition", targetPosition.inDegrees)
    CustomLogger.recordOutput("Climber/targetVoltage", targetVoltage.inVolts)
    CustomLogger.recordOutput("Climber/minAngleReached", minAngleReached)
    CustomLogger.recordOutput("Climber/maxAngleReached", maxAngleReached)

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
        val latched = targetPosition > inputs.climberPosition
        val feedforward =
          if (latched) ClimberTunableValues.kGLatched.get()
          else ClimberTunableValues.kGUnLatched.get()

        io.setPosition(targetPosition, feedforward)
        nextState = fromRequestToState(currentRequest)
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
