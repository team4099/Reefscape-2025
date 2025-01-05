package com.team4099.robot2023.subsystems.wrist

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRotations
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.perSecond

interface WristIO {
  class WristIOInputs : LoggableInputs {

    var wristPosition = 0.0.degrees
    var wristAbsoluteEncoderPosition = 0.0.degrees
    var wristVelocity = 0.0.degrees.perSecond
    var wristAppliedVoltage = 0.0.volts
    var wristDutyCycle = 0.0.volts
    var wristTorque = 0.0
    var wristStatorCurrent = 0.0.amps
    var wristSupplyCurrent = 0.0.amps
    var wristTemperature = 0.0.celsius

    var wristAcceleration = 0.0.degrees.perSecond.perSecond

    var isSimulated = false

    override fun toLog(table: LogTable) {
      table.put("wristPosition", wristPosition.inDegrees)
      table.put("wristAbsoluteEncoderRotations", wristAbsoluteEncoderPosition.inRotations)
      table.put("wristPositionRotations", wristPosition.inRotations)
      table.put("wristAbosluteEncoderPosition", wristAbsoluteEncoderPosition.inDegrees)
      table.put("wristVelocity", wristVelocity.inDegreesPerSecond)
      table.put("wristAppliedVoltage", wristAppliedVoltage.inVolts)
      table.put("wristStatorCurrent", wristStatorCurrent.inAmperes)
      table.put("wristSupplyCurrent", wristSupplyCurrent.inAmperes)
      table.put("wristTemperature", wristTemperature.inCelsius)
      table.put("wristAcceleration", wristAcceleration.inDegreesPerSecondPerSecond)
    }

    override fun fromLog(table: LogTable) {

      // wrist logs
      table.get("wristPostion", wristPosition.inDegrees).let { wristPosition = it.degrees }
      table.get("wristPostion", wristAbsoluteEncoderPosition.inDegrees).let {
        wristAbsoluteEncoderPosition = it.degrees
      }
      table.get("wristVelocity", wristVelocity.inDegreesPerSecond).let {
        wristVelocity = it.degrees.perSecond
      }
      table.get("wristAppliedVoltage", wristAppliedVoltage.inVolts).let {
        wristAppliedVoltage = it.volts
      }
      table.get("wristStatorCurrent", wristStatorCurrent.inAmperes).let {
        wristStatorCurrent = it.amps
      }
      table.get("wristSupplyCurrent", wristSupplyCurrent.inAmperes).let {
        wristSupplyCurrent = it.amps
      }
      table.get("wristTemperature", wristTemperature.inCelsius).let {
        wristTemperature = it.celsius
      }
      table.get("wristAcceleration", wristAcceleration.inDegreesPerSecondPerSecond).let {
        wristAcceleration = it.degrees.perSecond.perSecond
      }
    }
  }

  fun updateInputs(inputs: WristIOInputs) {}

  /*fun setRollerVoltage (voltage: ElectricalPotential){

  }*/
  fun setWristVoltage(voltage: ElectricalPotential) {}

  // fun setFeederVoltage (voltage: ElectricalPotential){

  //    }
  fun setWristPosition(position: Angle, feedforward: ElectricalPotential, travelingUp: Boolean) {}

  // fun setRollerBrakeMode (brake: Boolean){

  // }
  // fun setFeederBrakeMode (brake: Boolean){

  //  }
  fun setWristBrakeMode(brake: Boolean) {}

  fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}

  fun configPIDSlot1(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}

  fun configPIDSlot2(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}

  fun zeroEncoder() {}
}
