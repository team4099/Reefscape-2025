package com.team4099.robot2025.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
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
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecondPerSecond
import org.team4099.lib.units.derived.volts
import edu.wpi.first.units.measure.Angle as WPILibAngle
import edu.wpi.first.units.measure.AngularVelocity as WPILibAngularVelocity
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object ElevatorIOTalon : ElevatorIO {
  private val leaderTalon: TalonFX = TalonFX(Constants.Elevator.LEADER_MOTOR_ID)
  private val followerTalon: TalonFX = TalonFX(Constants.Elevator.FOLLOWER_MOTOR_ID)

  private val leaderConfigs: TalonFXConfiguration = TalonFXConfiguration()
  private val followerConfigs: TalonFXConfiguration = TalonFXConfiguration()

  private val leaderMotionMagicConfigs = leaderConfigs.MotionMagic

  val voltageControl: VoltageOut = VoltageOut(-1337.volts.inVolts)

  val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.inches.inInches)

  private val leaderSensor =
    ctreLinearMechanismSensor(
      leaderTalon,
      ElevatorConstants.GEAR_RATIO,
      ElevatorConstants.SPOOL_DIAMETER,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )

  private val followerSensor =
    ctreLinearMechanismSensor(
      followerTalon,
      ElevatorConstants.GEAR_RATIO,
      ElevatorConstants.SPOOL_DIAMETER,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )

  var leaderPositionStatusSignal: StatusSignal<WPILibAngle>
  var leaderVelocityStatusSignal: StatusSignal<WPILibAngularVelocity>
  var leaderStatorCurrentSignal: StatusSignal<WPILibCurrent>
  var leaderSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  var leaderTempSignal: StatusSignal<WPILibTemperature>
  var leaderDutyCycle: StatusSignal<Double>

  var followerStatorCurrentSignal: StatusSignal<WPILibCurrent>
  var followerSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  var followerTempSignal: StatusSignal<WPILibTemperature>
  var followerDutyCycle: StatusSignal<Double>
  var motorVoltage: StatusSignal<WPILibVoltage>
  var motorTorque: StatusSignal<WPILibCurrent>

  init {

    leaderTalon.clearStickyFaults()
    followerTalon.clearStickyFaults()

    leaderTalon.configurator.apply(TalonFXConfiguration())
    followerTalon.configurator.apply(TalonFXConfiguration())

    leaderTalon.clearStickyFaults()
    followerTalon.clearStickyFaults()

    leaderConfigs.Slot0.kP =
      leaderSensor.proportionalPositionGainToRawUnits(ElevatorConstants.PID.REAL_KP)
    leaderConfigs.Slot0.kI =
      leaderSensor.integralPositionGainToRawUnits(ElevatorConstants.PID.REAL_KI)
    leaderConfigs.Slot0.kD =
      leaderSensor.derivativePositionGainToRawUnits(ElevatorConstants.PID.REAL_KD)
    leaderConfigs.Slot0.kG = ElevatorConstants.PID.KG_FIRST_STAGE.inVolts
    leaderConfigs.Slot0.kS = ElevatorConstants.PID.REAL_KS_FIRST_STAGE.inVolts
    leaderConfigs.Slot0.kV = ElevatorConstants.PID.KV_FIRST_STAGE.inVoltsPerMeterPerSecond
    leaderConfigs.Slot0.kA = ElevatorConstants.PID.KA_FIRST_STAGE.inVoltsPerMeterPerSecondPerSecond
    leaderConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static

    leaderConfigs.Slot1.kP =
      leaderSensor.proportionalPositionGainToRawUnits(ElevatorConstants.PID.REAL_KP)
    leaderConfigs.Slot1.kI =
      leaderSensor.integralPositionGainToRawUnits(ElevatorConstants.PID.REAL_KI)
    leaderConfigs.Slot1.kD =
      leaderSensor.derivativePositionGainToRawUnits(ElevatorConstants.PID.REAL_KD)
    leaderConfigs.Slot1.kG = ElevatorConstants.PID.KG_SECOND_STAGE.inVolts
    leaderConfigs.Slot1.kS = ElevatorConstants.PID.REAL_KS_SECOND_STAGE.inVolts
    leaderConfigs.Slot1.kV = ElevatorConstants.PID.KV_SECOND_STAGE.inVoltsPerMeterPerSecond
    leaderConfigs.Slot1.kA = ElevatorConstants.PID.KA_SECOND_STAGE.inVoltsPerMeterPerSecondPerSecond
    leaderConfigs.Slot1.GravityType = GravityTypeValue.Elevator_Static

    followerConfigs.Slot0.kP =
      followerSensor.proportionalPositionGainToRawUnits(ElevatorConstants.PID.REAL_KP)
    followerConfigs.Slot0.kI =
      followerSensor.integralPositionGainToRawUnits(ElevatorConstants.PID.REAL_KI)
    followerConfigs.Slot0.kD =
      followerSensor.derivativePositionGainToRawUnits(ElevatorConstants.PID.REAL_KD)
    followerConfigs.Slot0.kG = ElevatorConstants.PID.KG_FIRST_STAGE.inVolts
    followerConfigs.Slot0.kS = ElevatorConstants.PID.REAL_KS_FIRST_STAGE.inVolts
    followerConfigs.Slot0.kV = ElevatorConstants.PID.KV_FIRST_STAGE.inVoltsPerMeterPerSecond
    followerConfigs.Slot0.kA =
      ElevatorConstants.PID.KA_FIRST_STAGE.inVoltsPerMeterPerSecondPerSecond
    followerConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static

    followerConfigs.Slot1.kP =
      followerSensor.proportionalPositionGainToRawUnits(ElevatorConstants.PID.REAL_KP)
    followerConfigs.Slot1.kI =
      followerSensor.integralPositionGainToRawUnits(ElevatorConstants.PID.REAL_KI)
    followerConfigs.Slot1.kD =
      followerSensor.derivativePositionGainToRawUnits(ElevatorConstants.PID.REAL_KD)
    followerConfigs.Slot1.kG = ElevatorConstants.PID.KG_SECOND_STAGE.inVolts
    followerConfigs.Slot1.kS = ElevatorConstants.PID.REAL_KS_SECOND_STAGE.inVolts
    followerConfigs.Slot1.kV = ElevatorConstants.PID.KV_SECOND_STAGE.inVoltsPerMeterPerSecond
    followerConfigs.Slot1.kA =
      ElevatorConstants.PID.KA_SECOND_STAGE.inVoltsPerMeterPerSecondPerSecond
    followerConfigs.Slot1.GravityType = GravityTypeValue.Elevator_Static

    leaderConfigs.CurrentLimits.SupplyCurrentLimit =
      ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT.inAmperes
    leaderConfigs.CurrentLimits.SupplyCurrentLowerLimit =
      ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT.inAmperes
    leaderConfigs.CurrentLimits.StatorCurrentLimit =
      ElevatorConstants.LEADER_STATOR_CURRENT_LIMIT.inAmperes
    leaderConfigs.CurrentLimits.SupplyCurrentLowerTime =
      ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT.inAmperes
    leaderConfigs.CurrentLimits.StatorCurrentLimitEnable = true
    leaderConfigs.CurrentLimits.SupplyCurrentLimitEnable = true
    leaderMotionMagicConfigs.MotionMagicCruiseVelocity = 80.0
    leaderMotionMagicConfigs.MotionMagicAcceleration = 160.0

    followerConfigs.CurrentLimits.SupplyCurrentLimit =
      ElevatorConstants.FOLLOWER_SUPPLY_CURRENT_LIMIT.inAmperes
    followerConfigs.CurrentLimits.SupplyCurrentLowerLimit =
      ElevatorConstants.FOLLOWER_SUPPLY_CURRENT_LIMIT.inAmperes
    followerConfigs.CurrentLimits.StatorCurrentLimit =
      ElevatorConstants.FOLLOWER_STATOR_CURRENT_LIMIT.inAmperes
    followerConfigs.CurrentLimits.SupplyCurrentLowerTime =
      ElevatorConstants.FOLLOWER_SUPPLY_CURRENT_LIMIT.inAmperes
    followerConfigs.CurrentLimits.StatorCurrentLimitEnable = true
    followerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true
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

  override fun configFirstStagePID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    val pidConfiguration = Slot0Configs()
    pidConfiguration.kP = leaderSensor.proportionalPositionGainToRawUnits(kP)
    pidConfiguration.kI = leaderSensor.integralPositionGainToRawUnits(kI)
    pidConfiguration.kD = leaderSensor.derivativePositionGainToRawUnits(kD)

    leaderTalon.configurator.apply(pidConfiguration)
  }

  override fun configSecondStagePID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    val pidConfiguration = Slot1Configs()
    pidConfiguration.kP = leaderSensor.proportionalPositionGainToRawUnits(kP)
    pidConfiguration.kI = leaderSensor.integralPositionGainToRawUnits(kI)
    pidConfiguration.kD = leaderSensor.derivativePositionGainToRawUnits(kD)

    leaderTalon.configurator.apply(pidConfiguration)
  }

  override fun setPosition(position: Length) {
    val slot = if (leaderSensor.position > ElevatorConstants.FIRST_STAGE_HEIGHT) 1 else 0

    leaderTalon.setControl(
      motionMagicControl.withPosition(leaderSensor.positionToRawUnits(position)).withSlot(slot)
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
