package com.team4099.robot2025.subsystems.elevator

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.perSecond

interface ElevatorIO {
  class ElevatorInputs : LoggableInputs {
    var elevatorPosition = 0.0.inches
    var elevatorVelocity = 0.0.inches.perSecond

    var leaderAppliedVoltage = 0.0.volts
    var followerAppliedVoltage = 0.0.volts

    var leaderTemperature = 0.0.celsius
    var followerTemperature = 0.0.celsius

    var leaderSupplyCurrent = 0.0.amps
    var followerSupplyCurrent = 0.0.amps

    var leaderStatorCurrent = 0.0.amps
    var followerStatorCurrent = 0.0.amps

    var isSimulating = false

    override fun toLog(table: LogTable?) {
      table?.put("elevatorPositionInches", elevatorPosition.inInches)
      table?.put("elevatorVelocityInchesPerSecond", elevatorVelocity.inInchesPerSecond)
      table?.put("elevatorLeaderAppliedVoltage", leaderAppliedVoltage.inVolts)
      table?.put("elevatorFollowerAppliedVoltage", followerAppliedVoltage.inVolts)
      table?.put("elevatorLeaderTempCelsius", leaderTemperature.inCelsius)
      table?.put("elevatorFollowerTempCelsius", followerTemperature.inCelsius)
      table?.put("elevatorLeaderSupplyCurrent", leaderSupplyCurrent.inAmperes)
      table?.put("elevatorFollowerSupplyCurrent", followerSupplyCurrent.inAmperes)
      table?.put("elevatorLeaderStatorCurrent", leaderStatorCurrent.inAmperes)
      table?.put("elevatorFollowerStatorCurrent", followerStatorCurrent.inAmperes)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("elevatorPositionInches", elevatorPosition.inInches)?.let {
        elevatorPosition = it.inches
      }
      table?.get("elevatorVelocityInchesPerSecond", elevatorVelocity.inInchesPerSecond)?.let {
        elevatorVelocity = it.inches.perSecond
      }
      table?.get("elevatorLeaderAppliedVoltage", leaderAppliedVoltage.inVolts)?.let {
        leaderAppliedVoltage = it.volts
      }
      table?.get("elevatorFollowerAppliedVoltage", followerAppliedVoltage.inVolts)?.let {
        followerAppliedVoltage = it.volts
      }
      table?.get("elevatorLeaderTempCelsius", leaderTemperature.inCelsius)?.let {
        leaderTemperature = it.celsius
      }
      table?.get("elevatorFollowerTempCelsius", followerTemperature.inCelsius)?.let {
        followerTemperature = it.celsius
      }
      table?.get("elevatorLeaderSupplyCurrent", leaderSupplyCurrent.inAmperes)?.let {
        leaderSupplyCurrent = it.amps
      }
      table?.get("elevatorFollowerSupplyCurrent", followerSupplyCurrent.inAmperes)?.let {
        followerSupplyCurrent = it.amps
      }
      table?.get("elevatorLeaderSupplyCurrent", leaderStatorCurrent.inAmperes)?.let {
        leaderStatorCurrent = it.amps
      }
      table?.get("elevatorFollowerSupplyCurrent", followerStatorCurrent.inAmperes)?.let {
        followerStatorCurrent = it.amps
      }
    }
  }
  fun updateInputs(inputs: ElevatorInputs) {}
  fun setPosition(position: Length) {}
  fun setVoltage(targetVoltage: ElectricalPotential) {}
  fun zeroEncoder() {}

  fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {}
}
