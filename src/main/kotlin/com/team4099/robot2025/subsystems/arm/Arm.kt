package com.team4099.robot2025.subsystems.arm

import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.subsystems.arm.ArmTunableValues.armKD
import com.team4099.robot2025.subsystems.arm.ArmTunableValues.armKI
import com.team4099.robot2025.subsystems.arm.ArmTunableValues.armKP
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Arm(private val io: ArmIO) {
  val inputs = ArmIO.ArmIOInputs()

  private var armTargetVoltage: ElectricalPotential = 0.0.volts

  private var currentState = ArmState.UNINITIALIZED
  var currentRequest: Request.ArmRequest = Request.ArmRequest.OpenLoop(0.volts)
    set(value) {
      when (value) {
        is Request.ArmRequest.OpenLoop -> {
          armTargetVoltage = value.armVoltage
        }
        else -> {}
      }
      field = value
    }

  fun periodic() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Arm", inputs)
    CustomLogger.recordOutput("Arm/currentState", currentState.toString())


    var nextState = currentState
    CustomLogger.recordOutput("Arm/nextState", nextState.toString())
    CustomLogger.recordOutput("Arm/armTargetVoltage", armTargetVoltage.inVolts)
    when (currentState) {
      ArmState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      ArmState.OPEN_LOOP -> {
        io.setArmVoltage(armTargetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
    }

    currentState = nextState
  }

  companion object {
    enum class ArmState {
      UNINITIALIZED,
      OPEN_LOOP
    }

    // Translates Current Request to a State
    fun fromRequestToState(request: Request.ArmRequest): ArmState {
      return when (request) {
        is Request.ArmRequest.OpenLoop -> ArmState.OPEN_LOOP
      }
    }
  }
}
