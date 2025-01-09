package com.team4099.robot2025.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.ElevatorConstants
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
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

  val voltageControl: VoltageOut = VoltageOut(-1337.volts.inVolts)

  val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.inches.inInches)

  private val leaderSensor =
    ctreLinearMechanismSensor(
      leaderTalon,
      ElevatorConstants.ELEVATOR_PULLEY_TO_MOTOR,
      ElevatorConstants.SPOOL_DIAMETER,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )

  private val followerSensor =
    ctreLinearMechanismSensor(
      followerTalon,
      ElevatorConstants.ELEVATOR_PULLEY_TO_MOTOR,
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

    followerConfigs.Slot0.kP =
      followerSensor.proportionalPositionGainToRawUnits(ElevatorConstants.PID.REAL_KP)
    followerConfigs.Slot0.kI =
      followerSensor.integralPositionGainToRawUnits(ElevatorConstants.PID.REAL_KI)
    followerConfigs.Slot0.kD =
      followerSensor.derivativePositionGainToRawUnits(ElevatorConstants.PID.REAL_KD)

    leaderConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT
    leaderConfigs.CurrentLimits.SupplyCurrentLowerLimit =
      ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT
    leaderConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.LEADER_STATOR_CURRENT_LIMIT
    leaderConfigs.CurrentLimits.SupplyCurrentLowerTime =
      ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT
    leaderConfigs.CurrentLimits.StatorCurrentLimitEnable = true
    leaderConfigs.CurrentLimits.SupplyCurrentLimitEnable = true

    followerConfigs.CurrentLimits.SupplyCurrentLimit =
      ElevatorConstants.FOLLOWER_SUPPLY_CURRENT_LIMIT
    followerConfigs.CurrentLimits.SupplyCurrentLowerLimit =
      ElevatorConstants.FOLLOWER_SUPPLY_CURRENT_LIMIT
    followerConfigs.CurrentLimits.StatorCurrentLimit =
      ElevatorConstants.FOLLOWER_STATOR_CURRENT_LIMIT
    followerConfigs.CurrentLimits.SupplyCurrentLowerTime =
      ElevatorConstants.FOLLOWER_SUPPLY_CURRENT_LIMIT
    followerConfigs.CurrentLimits.StatorCurrentLimitEnable = true
    followerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true

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

    var slot = if (leaderSensor.position > ElevatorConstants.FIRST_STAGE_HEIGHT) 1 else 0

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
