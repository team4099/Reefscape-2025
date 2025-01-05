package com.team4099.robot2023.subsystems.elevator

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.subsystems.falconspin.Falcon500
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

object ElevatorIOKraken : ElevatorIO {
  private val elevatorLeaderKraken: TalonFX = TalonFX(Constants.Elevator.LEADER_MOTOR_ID)
  private val elevatorFollowerKraken: TalonFX = TalonFX(Constants.Elevator.FOLLOWER_MOTOR_ID)
  private val leaderSensor =
    ctreLinearMechanismSensor(
      elevatorLeaderKraken,
      ElevatorConstants.ELEVATOR_PULLEY_TO_MOTOR,
      ElevatorConstants.SPOOL_DIAMETER,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )
  private val followerSensor =
    ctreLinearMechanismSensor(
      elevatorLeaderKraken,
      ElevatorConstants.ELEVATOR_PULLEY_TO_MOTOR,
      ElevatorConstants.SPOOL_DIAMETER,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )
  private val elevatorLeaderConfiguration: TalonFXConfiguration = TalonFXConfiguration()
  private val elevatorFollowerConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  var elevatorLeaderStatorCurrentSignal: StatusSignal<Double>
  var elevatorLeaderSupplyCurrentSignal: StatusSignal<Double>
  var elevatorLeadertempSignal: StatusSignal<Double>
  var elevatorLeaderDutyCycle: StatusSignal<Double>
  var elevatorFollowerStatorCurrentSignal: StatusSignal<Double>
  var elevatorFollowerSupplyCurrentSignal: StatusSignal<Double>
  var elevatorFollowertempSignal: StatusSignal<Double>
  var elevatorFollowerDutyCycle: StatusSignal<Double>

  init {
    elevatorLeaderKraken.clearStickyFaults()
    elevatorFollowerKraken.clearStickyFaults()
    elevatorLeaderConfiguration.Slot0.kP =
      leaderSensor.proportionalPositionGainToRawUnits(ElevatorConstants.LEADER_KP)
    elevatorLeaderConfiguration.Slot0.kI =
      leaderSensor.integralPositionGainToRawUnits(ElevatorConstants.LEADER_KI)
    elevatorLeaderConfiguration.Slot0.kD =
      leaderSensor.derivativePositionGainToRawUnits(ElevatorConstants.LEADER_KD)

    elevatorFollowerConfiguration.Slot0.kP =
      followerSensor.proportionalPositionGainToRawUnits(ElevatorConstants.FOLLOWER_KP)
    elevatorFollowerConfiguration.Slot0.kI =
      followerSensor.integralPositionGainToRawUnits(ElevatorConstants.FOLLOWER_KI)
    elevatorFollowerConfiguration.Slot0.kD =
      followerSensor.derivativePositionGainToRawUnits(ElevatorConstants.FOLLOWER_KD)

    elevatorLeaderConfiguration.CurrentLimits.SupplyCurrentLimit =
      ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT.inAmperes
    elevatorLeaderConfiguration.CurrentLimits.SupplyCurrentThreshold =
      ElevatorConstants.LEADER_THRESHOLD_CURRENT_LIMIT.inAmperes
    elevatorLeaderConfiguration.CurrentLimits.SupplyTimeThreshold =
      ElevatorConstants.LEADER_SUPPLY_TIME_THRESHOLD.inSeconds
    elevatorLeaderConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    elevatorLeaderConfiguration.CurrentLimits.StatorCurrentLimit =
      ElevatorConstants.LEADER_STATOR_CURRENT_LIMIT.inAmperes
    elevatorLeaderConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

    elevatorFollowerConfiguration.CurrentLimits.SupplyCurrentLimit =
      ElevatorConstants.FOLLOWER_SUPPLY_CURRENT_LIMIT.inAmperes
    elevatorFollowerConfiguration.CurrentLimits.SupplyCurrentThreshold =
      ElevatorConstants.FOLLOWER_THRESHOLD_CURRENT_LIMIT.inAmperes
    elevatorFollowerConfiguration.CurrentLimits.SupplyTimeThreshold =
      ElevatorConstants.FOLLOWER_SUPPLY_TIME_THRESHOLD.inSeconds
    elevatorFollowerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    elevatorFollowerConfiguration.CurrentLimits.StatorCurrentLimit =
      ElevatorConstants.FOLLOWER_STATOR_CURRENT_LIMIT.inAmperes
    elevatorFollowerConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

    elevatorLeaderConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    elevatorFollowerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    elevatorLeaderKraken.configurator.apply(elevatorLeaderConfiguration)
    elevatorFollowerKraken.configurator.apply(elevatorFollowerConfiguration)
    elevatorLeaderKraken.inverted = true
    elevatorFollowerKraken.inverted = false
    MotorChecker.add(
      "Elevator",
      "Extension",
      MotorCollection(
        mutableListOf(
          Falcon500(elevatorLeaderKraken, "Leader Extension Motor"),
          (Falcon500(elevatorFollowerKraken, "Follower Extension Motor"))
        ),
        ElevatorConstants.LEADER_STATOR_CURRENT_LIMIT,
        30.celsius,
        ElevatorConstants.LEADER_STATOR_CURRENT_LIMIT - 30.amps,
        110.celsius
      )
    )
    elevatorLeaderStatorCurrentSignal = elevatorLeaderKraken.statorCurrent
    elevatorLeaderSupplyCurrentSignal = elevatorLeaderKraken.supplyCurrent
    elevatorLeadertempSignal = elevatorLeaderKraken.deviceTemp
    elevatorLeaderDutyCycle = elevatorLeaderKraken.dutyCycle
    elevatorFollowerStatorCurrentSignal = elevatorFollowerKraken.statorCurrent
    elevatorFollowerSupplyCurrentSignal = elevatorFollowerKraken.supplyCurrent
    elevatorFollowertempSignal = elevatorFollowerKraken.deviceTemp
    elevatorFollowerDutyCycle = elevatorFollowerKraken.dutyCycle
  }

  override fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    val pidConfiguration = Slot0Configs()
    pidConfiguration.kP = leaderSensor.proportionalPositionGainToRawUnits(kP)
    pidConfiguration.kI = leaderSensor.integralPositionGainToRawUnits(kI)
    pidConfiguration.kD = leaderSensor.derivativePositionGainToRawUnits(kD)

    elevatorLeaderKraken.configurator.apply(pidConfiguration)
  }

  override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {
    inputs.elevatorPosition = leaderSensor.position
    inputs.elevatorVelocity = leaderSensor.velocity
    inputs.leaderAppliedVoltage = elevatorLeaderDutyCycle.value.volts
    inputs.followerAppliedVoltage = elevatorFollowerDutyCycle.value.volts
    inputs.leaderSupplyCurrent = elevatorLeaderSupplyCurrentSignal.value.amps
    inputs.leaderStatorCurrent = elevatorLeaderStatorCurrentSignal.value.amps
    inputs.followerSupplyCurrent = elevatorFollowerSupplyCurrentSignal.value.amps
    inputs.followerStatorCurrent = elevatorFollowerStatorCurrentSignal.value.amps
    inputs.leaderTempCelcius = elevatorLeadertempSignal.value.celsius
    inputs.followerTempCelcius = elevatorFollowertempSignal.value.celsius
  }

  override fun setOutputVoltage(voltage: ElectricalPotential) {
    if (((leaderSensor.position < 0.5.inches) && (voltage < 0.volts)) ||
      (
        leaderSensor.position > ElevatorConstants.ELEVATOR_MAX_EXTENSION - 0.5.inches &&
          (voltage > 0.volts)
        )
    ) {
      elevatorLeaderKraken.setVoltage(0.0)
    } else {
      elevatorLeaderKraken.setVoltage(voltage.inVolts)
    }
  }

  override fun setPosition(position: Length, feedForward: ElectricalPotential) {
    elevatorLeaderKraken.setControl(
      PositionVoltage(
        leaderSensor.getRawPosition(), 0.0, true, feedForward.inVolts, 0, false, false, false
      )
    )
  }

  override fun zeroEncoder() {
    elevatorLeaderKraken.setPosition(0.0)
    elevatorFollowerKraken.setPosition(0.0)
  }
}
