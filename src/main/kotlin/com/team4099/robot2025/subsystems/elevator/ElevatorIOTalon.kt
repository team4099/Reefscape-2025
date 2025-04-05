package com.team4099.robot2025.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.config.constants.ElevatorConstants.MAX_ACCELERATION
import com.team4099.robot2025.config.constants.ElevatorConstants.MAX_VELOCITY
import edu.wpi.first.units.measure.AngularVelocity
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecond
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond
import kotlin.time.times
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

  private val voltageControl: VoltageOut = VoltageOut(-1337.volts.inVolts)
  private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.inches.inInches)
  private val positionControl: PositionVoltage = PositionVoltage(-1337.inches.inInches)

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
  private var leaderPositionSignal: StatusSignal<edu.wpi.first.units.measure.Angle>
  private var leaderVelocitySignal: StatusSignal<AngularVelocity>

  private var followerStatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerTempSignal: StatusSignal<WPILibTemperature>
  private var followerDutyCycle: StatusSignal<Double>
  private var motorVoltage: StatusSignal<WPILibVoltage>
  private var motorTorque: StatusSignal<WPILibCurrent>

  private var motionMagicTargetVelocity: StatusSignal<Double>
  private var motionMagicTargetPosition: StatusSignal<Double>

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

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    // configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
    // configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true

    // configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    // leaderSensor.positionToRawUnits(ElevatorConstants.UPWARDS_EXTENSION_LIMIT)

    // configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    // leaderSensor.positionToRawUnits(ElevatorConstants.DOWNWARDS_EXTENSION_LIMIT)

    configs.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY.inInchesPerSecond
    configs.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION.inInchesPerSecondPerSecond

    followerTalon.setControl(Follower(Constants.Elevator.LEADER_MOTOR_ID, true))

    leaderPositionSignal = leaderTalon.position
    leaderVelocitySignal = leaderTalon.velocity
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

    motionMagicTargetPosition = leaderTalon.closedLoopReference
    motionMagicTargetVelocity = leaderTalon.closedLoopReferenceSlope

    motionMagicTargetPosition.setUpdateFrequency(250.0)
    motionMagicTargetVelocity.setUpdateFrequency(250.0)

    leaderTalon.configurator.apply(configs)
    followerTalon.configurator.apply(configs)
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
      motorTorque,
      motorVoltage,
      leaderPositionSignal,
      leaderVelocitySignal,
      leaderTempSignal,
      leaderDutyCycle,
      leaderStatorCurrentSignal,
      leaderSupplyCurrentSignal,
      followerTempSignal,
      followerDutyCycle,
      followerStatorCurrentSignal,
      followerSupplyCurrentSignal,
      motionMagicTargetPosition,
      motionMagicTargetVelocity
    )
  }

  override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {
    updateSignals()

    inputs.elevatorPosition = leaderSensor.position
    inputs.elevatorVelocity = leaderSensor.velocity

    inputs.leaderTemperature = leaderTempSignal.valueAsDouble.celsius
    inputs.leaderSupplyCurrent = leaderSupplyCurrentSignal.valueAsDouble.amps
    inputs.leaderStatorCurrent = leaderStatorCurrentSignal.valueAsDouble.amps
    inputs.leaderAppliedVoltage = (leaderDutyCycle.valueAsDouble * 12).volts

    inputs.followerTemperature = followerTempSignal.valueAsDouble.celsius
    inputs.followerStatorCurrent = followerStatorCurrentSignal.valueAsDouble.amps
    inputs.followerSupplyCurrent = followerSupplyCurrentSignal.valueAsDouble.amps
    inputs.followerAppliedVoltage = (followerDutyCycle.valueAsDouble * 12).volts

    Logger.recordOutput(
      "Elevator/motionMagicPositon",
      motionMagicTargetPosition.value *
        ElevatorConstants.GEAR_RATIO *
        (Math.PI * ElevatorConstants.SPOOL_DIAMETER.inInches)
    )
    Logger.recordOutput(
      "Elevator/motionMagicVelocity",
      motionMagicTargetVelocity.value *
        ElevatorConstants.GEAR_RATIO *
        (Math.PI * ElevatorConstants.SPOOL_DIAMETER.inInches)
    )
  }

  override fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    slot0Configs.kP = kP.inVoltsPerInch
    slot0Configs.kI = kI.inVoltsPerInchSeconds
    slot0Configs.kD = kD.inVoltsPerInchPerSecond

    slot1Configs.kP = kP.inVoltsPerInch
    slot1Configs.kI = kI.inVoltsPerInchSeconds
    slot1Configs.kD = kD.inVoltsPerInchPerSecond

    slot2Configs.kP = kP.inVoltsPerInch
    slot2Configs.kI = kI.inVoltsPerInchSeconds
    slot2Configs.kD = kD.inVoltsPerInchPerSecond

    leaderTalon.configurator.apply(slot0Configs)
    followerTalon.configurator.apply(slot0Configs)
    leaderTalon.configurator.apply(slot1Configs)
    followerTalon.configurator.apply(slot1Configs)
    leaderTalon.configurator.apply(slot2Configs)
    followerTalon.configurator.apply(slot2Configs)
  }

  override fun configFF(
    kGFirstStage: ElectricalPotential,
    kGSecondStage: ElectricalPotential,
    kGThirdStage: ElectricalPotential,
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Meter, Volt>,
    kA: AccelerationFeedforward<Meter, Volt>
  ) {
    slot0Configs.kG = kGFirstStage.inVolts
    slot0Configs.kS = kS.inVolts
    slot0Configs.kV = kV.inVoltsPerInchPerSecond
    slot0Configs.kA = kA.inVoltsPerMetersPerSecondPerSecond
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static

    slot1Configs.kG = kGSecondStage.inVolts
    slot1Configs.kS = kS.inVolts
    slot1Configs.kV = kV.inVoltsPerInchPerSecond
    slot1Configs.kA = kA.inVoltsPerMetersPerSecondPerSecond
    slot1Configs.GravityType = GravityTypeValue.Elevator_Static

    slot2Configs.kG = kGThirdStage.inVolts
    slot2Configs.kS = kS.inVolts
    slot2Configs.kV =
      kV.inVoltsPerInchPerSecond + ElevatorConstants.PID.KV_ADD.inVoltsPerMetersPerSecond
    slot2Configs.kA = kA.inVoltsPerMetersPerSecondPerSecond
    slot2Configs.GravityType = GravityTypeValue.Elevator_Static

    leaderTalon.configurator.apply(slot0Configs)
    followerTalon.configurator.apply(slot0Configs)
  }

  override fun setPosition(position: Length) {
    val slotUsed =
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
        .withSlot(slotUsed)
    )
  }

  override fun setVoltage(targetVoltage: ElectricalPotential) {
    leaderTalon.setControl(VoltageOut(targetVoltage.inVolts))
  }

  override fun zeroEncoder() {
    leaderTalon.setPosition(0.0)
    followerTalon.setPosition(0.0)
  }
}
