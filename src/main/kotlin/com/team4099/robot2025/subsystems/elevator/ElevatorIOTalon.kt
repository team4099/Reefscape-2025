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
  private var slot1Configs = configs.Slot1
  private var slot2Configs = configs.Slot2
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

  private var leaderPositionStatusSignal: StatusSignal<WPILibAngle>
  private var leaderVelocityStatusSignal: StatusSignal<WPILibAngularVelocity>
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

    slot0Configs.kP = ElevatorConstants.PID.REAL_KP.inVoltsPerMeter
    slot0Configs.kI = ElevatorConstants.PID.REAL_KI.inVoltsPerMeterSeconds
    slot0Configs.kD = ElevatorConstants.PID.REAL_KD.inVoltsPerMeterPerSecond

    slot1Configs.kP = slot0Configs.kP
    slot1Configs.kI = slot0Configs.kP
    slot1Configs.kD = slot0Configs.kD
    slot2Configs.kP = slot0Configs.kP
    slot2Configs.kI = slot0Configs.kP
    slot2Configs.kD = slot0Configs.kD

    slot0Configs.kG = ElevatorConstants.PID.KG_FIRST_STAGE.inVolts
    slot0Configs.kS = ElevatorConstants.PID.REAL_KS_FIRST_STAGE.inVolts
    slot0Configs.kV = ElevatorConstants.PID.KV_FIRST_STAGE.inVoltsPerMeterPerSecond
    slot0Configs.kA = ElevatorConstants.PID.KA_FIRST_STAGE.inVoltsPerMeterPerSecondPerSecond

    slot1Configs.kG = ElevatorConstants.PID.KG_SECOND_STAGE.inVolts
    slot1Configs.kS = ElevatorConstants.PID.REAL_KS_SECOND_STAGE.inVolts
    slot1Configs.kV = ElevatorConstants.PID.KV_SECOND_STAGE.inVoltsPerMeterPerSecond
    slot1Configs.kA = ElevatorConstants.PID.KA_SECOND_STAGE.inVoltsPerMeterPerSecondPerSecond

    slot2Configs.kG = ElevatorConstants.PID.KG_THIRD_STAGE.inVolts
    slot2Configs.kS = ElevatorConstants.PID.REAL_KS_THIRD_STAGE.inVolts
    slot2Configs.kV = ElevatorConstants.PID.KV_THIRD_STAGE.inVoltsPerMeterPerSecond
    slot2Configs.kA = ElevatorConstants.PID.KA_THIRD_STAGE.inVoltsPerMeterPerSecondPerSecond

    slot0Configs.GravityType = GravityTypeValue.Elevator_Static
    slot1Configs.GravityType = GravityTypeValue.Elevator_Static
    slot2Configs.GravityType = GravityTypeValue.Elevator_Static

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

    motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MAX_VELOCITY.inMetersPerSecond
    motionMagicConfigs.MotionMagicAcceleration =
      ElevatorConstants.MAX_ACCELERATION.inMetersPerSecondPerSecond
    motionMagicConfigs.MotionMagicExpo_kA =
      ElevatorConstants.PID.KA_FIRST_STAGE.inVoltsPerMeterPerSecondPerSecond

    followerTalon.setControl(Follower(Constants.Elevator.LEADER_MOTOR_ID, false))

    leaderPositionStatusSignal = leaderTalon.getPosition()
    leaderVelocityStatusSignal = leaderTalon.getVelocity()
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
    slot0Configs.kP = kP.inVoltsPerMeter
    slot0Configs.kI = kI.inVoltsPerMeterSeconds
    slot0Configs.kD = kD.inVoltsPerMeterPerSecond

    slot1Configs.kP = slot0Configs.kP
    slot1Configs.kI = slot0Configs.kP
    slot1Configs.kD = slot0Configs.kD

    slot2Configs.kP = slot0Configs.kP
    slot2Configs.kI = slot0Configs.kP
    slot2Configs.kD = slot0Configs.kD

    updateConfigs()
  }

  override fun configFFFirstStage(
    kG: ElectricalPotential,
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Meter, Volt>,
    kA: AccelerationFeedforward<Meter, Volt>
  ) {
    slot0Configs.kG = kG.inVolts
    slot0Configs.kS = kS.inVolts
    slot0Configs.kV = kV.inVoltsPerMeterPerSecond
    slot0Configs.kA = kA.inVoltsPerMeterPerSecondPerSecond
    updateConfigs()
  }

  override fun configFFSecondStage(
    kG: ElectricalPotential,
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Meter, Volt>,
    kA: AccelerationFeedforward<Meter, Volt>
  ) {
    slot1Configs.kG = kG.inVolts
    slot1Configs.kS = kS.inVolts
    slot1Configs.kV = kV.inVoltsPerMeterPerSecond
    slot1Configs.kA = kA.inVoltsPerMeterPerSecondPerSecond
    updateConfigs()
  }

  override fun configFFThirdStage(
    kG: ElectricalPotential,
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Meter, Volt>,
    kA: AccelerationFeedforward<Meter, Volt>
  ) {
    slot2Configs.kG = kG.inVolts
    slot2Configs.kS = kS.inVolts
    slot2Configs.kV = kV.inVoltsPerMeterPerSecond
    slot2Configs.kA = kA.inVoltsPerMeterPerSecondPerSecond
    updateConfigs()
  }

  override fun setPosition(position: Length) {

    val slot: Int =
      if (leaderSensor.position > ElevatorConstants.SECOND_STAGE_HEIGHT) {
        2
      } else if (leaderSensor.position > ElevatorConstants.FIRST_STAGE_HEIGHT) {
        1
      } else {
        0
      }

    leaderTalon.setControl(
      motionMagicControl
        .withPosition(leaderSensor.positionToRawUnits(position))
        .withSlot(slot)
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

  private fun updateConfigs() {
    leaderTalon.configurator.apply(configs)
    followerTalon.configurator.apply(configs)
  }
}
