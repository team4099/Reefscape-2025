package com.team4099.robot2025.subsystems.climber

import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Climber(private val io: ClimberIO) {
  val inputs: ClimberIO.ClimberInputs = ClimberIO.ClimberInputs()
  private var currentState: ClimberState = ClimberState.UNINITIALIZED

  private var targetVoltage: ElectricalPotential = 0.volts

  var currentRequest: Request.ClimberRequest = Request.ClimberRequest.Home()
    set(value) {
      when (value) {
        is Request.ClimberRequest.OpenLoop -> {
          targetVoltage = value.voltage
        }
        else -> {}
      }

      field = value
    }

  var isAtTargetedPosition: Boolean =
    (inputs.climberPosition - 90.degrees).absoluteValue <= 3.degrees

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
    CustomLogger.recordOutput("Climber/targetVoltage", targetVoltage.inVolts)

    var nextState = currentState

    when (currentState) {
      ClimberState.UNINITIALIZED -> {
        io.zeroEncoder()
        nextState = fromRequestToState(currentRequest)
      }
      ClimberState.OPEN_LOOP -> {
        setVoltage(targetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
    }

    currentState = nextState
  }

  private fun setVoltage(voltage: ElectricalPotential) {
    //    if (isHomed && isOutOfBounds(voltage)) {
    //      io.setVoltage(0.volts)
    //    } else {
    io.setVoltage(voltage)
    // }
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
