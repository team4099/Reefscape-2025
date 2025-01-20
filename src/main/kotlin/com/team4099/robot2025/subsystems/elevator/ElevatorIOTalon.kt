package com.team4099.robot2025.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.config.constants.ElevatorConstants.MAX_ACCELERATION
import com.team4099.robot2025.config.constants.ElevatorConstants.MAX_VELOCITY
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerMeter
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeterSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import edu.wpi.first.units.measure.Angle as WPILibAngle
import edu.wpi.first.units.measure.AngularVelocity as WPILibAngularVelocity
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object ElevatorIOTalon : ElevatorIO {
  private val leaderTalon: TalonFX = TalonFX(Constants.Elevator.LEADER_MOTOR_ID)
  private val followerTalon: TalonFX = TalonFX(Constants.Elevator.FOLLOWER_MOTOR_ID)

  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  private var slot0Configs = configs.Slot0
  private var motionMagicConfigs: MotionMagicConfigs = configs.MotionMagic

  private val voltageControl: VoltageOut = VoltageOut(-1337.volts.inVolts)
  private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.inches.inInches)

  private val leaderSensor =
    ctreLinearMechanismSensor(
      leaderTalon,
      ElevatorConstants.GEAR_RATIO,
      ElevatorConstants.SPOOL_DIAMETER,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )

  private var leaderStatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderTempSignal: StatusSignal<WPILibTemperature>
  private var leaderDutyCycle: StatusSignal<Double>

  private var followerStatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerTempSignal: StatusSignal<WPILibTemperature>
  private var followerDutyCycle: StatusSignal<Double>
  private var motorVoltage: StatusSignal<WPILibVoltage>
  private var motorTorque: StatusSignal<WPILibCurrent>

  init {
    leaderTalon.clearStickyFaults()
    followerTalon.clearStickyFaults()

    configs.CurrentLimits.SupplyCurrentLimit =
      ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLowerLimit =
      ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimit =
      ElevatorConstants.LEADER_STATOR_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLowerTime =
      ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.CurrentLimits.SupplyCurrentLimitEnable = true

    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
      leaderSensor.positionToRawUnits(ElevatorConstants.UPWARDS_EXTENSION_LIMIT)
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
      leaderSensor.positionToRawUnits(ElevatorConstants.DOWNWARDS_EXTENSION_LIMIT)

    motionMagicConfigs.MotionMagicCruiseVelocity = leaderSensor.velocityToRawUnits(MAX_VELOCITY)
    motionMagicConfigs.MotionMagicAcceleration =
      leaderSensor.accelerationToRawUnits(MAX_ACCELERATION)

    followerTalon.setControl(Follower(Constants.Elevator.LEADER_MOTOR_ID, true))

    leaderStatorCurrentSignal = leaderTalon.statorCurrent
    leaderSupplyCurrentSignal = leaderTalon.supplyCurrent
    leaderTempSignal = leaderTalon.deviceTemp
    leaderDutyCycle = leaderTalon.dutyCycle

    followerStatorCurrentSignal = followerTalon.statorCurrent
    followerSupplyCurrentSignal = followerTalon.supplyCurrent
    followerTempSignal = followerTalon.deviceTemp
    followerDutyCycle = followerTalon.dutyCycle

    motorVoltage = leaderTalon.motorVoltage
    motorTorque = leaderTalon.torqueCurrent

    leaderTalon.configurator.apply(configs)
    followerTalon.configurator.apply(configs)
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
      motorTorque,
      motorVoltage,
      leaderTempSignal,
      leaderDutyCycle,
      leaderStatorCurrentSignal,
      leaderSupplyCurrentSignal,
      followerTempSignal,
      followerDutyCycle,
      followerStatorCurrentSignal,
      followerSupplyCurrentSignal,
    )
  }

  override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {
    updateSignals()

    inputs.elevatorPosition = leaderSensor.position
    inputs.elevatorVelocity = leaderSensor.velocity

    inputs.leaderTemperature = leaderTalon.deviceTemp.valueAsDouble.celsius
    inputs.leaderSupplyCurrent = leaderTalon.supplyCurrent.valueAsDouble.amps
    inputs.leaderStatorCurrent = leaderTalon.statorCurrent.valueAsDouble.amps
    inputs.leaderAppliedVoltage = leaderTalon.dutyCycle.valueAsDouble.volts

    inputs.followerTemperature = followerTalon.deviceTemp.valueAsDouble.celsius
    inputs.followerStatorCurrent = followerTalon.statorCurrent.valueAsDouble.amps
    inputs.followerSupplyCurrent = followerTalon.supplyCurrent.valueAsDouble.amps
    inputs.followerAppliedVoltage = followerTalon.dutyCycle.valueAsDouble.volts
  }

  override fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    slot0Configs.kP = leaderSensor.proportionalPositionGainToRawUnits(kP)
    slot0Configs.kI = leaderSensor.integralPositionGainToRawUnits(kI)
    slot0Configs.kD = leaderSensor.derivativePositionGainToRawUnits(kD)


    leaderTalon.configurator.apply(slot0Configs)
    followerTalon.configurator.apply(slot0Configs)
  }

  override fun configFF(
    kG: ElectricalPotential,
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Meter, Volt>,
    kA: AccelerationFeedforward<Meter, Volt>
  ) {
    slot0Configs.kG = kG.inVolts
    slot0Configs.kS = kS.inVolts
    slot0Configs.kV = leaderSensor.velocityFeedforwardToRawUnits(kV)
    slot0Configs.kA = leaderSensor.accelerationFeedforwardToRawUnits(kA)
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static

    leaderTalon.configurator.apply(slot0Configs)
    followerTalon.configurator.apply(slot0Configs)
  }


  override fun setPosition(position: Length) {

    val feedforward: ElectricalPotential =
      if (leaderSensor.position > ElevatorConstants.SECOND_STAGE_HEIGHT) {
        ElevatorConstants.PID.KG_THIRD_STAGE
      } else if (leaderSensor.position > ElevatorConstants.FIRST_STAGE_HEIGHT) {
        ElevatorConstants.PID.KG_SECOND_STAGE
      } else {
        ElevatorConstants.PID.KG_FIRST_STAGE
      }

    leaderTalon.setControl(
      motionMagicControl
        .withPosition(leaderSensor.positionToRawUnits(position))
        .withFeedForward(feedforward.inVolts)
        .withLimitForwardMotion(true)
        .withLimitReverseMotion(true)
    )
  }

  override fun setVoltage(targetVoltage: ElectricalPotential) {
    leaderTalon.setControl(voltageControl.withOutput(targetVoltage.inVolts))
  }

  override fun zeroEncoder() {
    leaderTalon.setPosition(0.0)
    followerTalon.setPosition(0.0)
  }

}
