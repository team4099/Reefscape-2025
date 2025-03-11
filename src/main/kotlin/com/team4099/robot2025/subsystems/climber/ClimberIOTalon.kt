package com.team4099.robot2025.subsystems.climber

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.newtons
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ClimberIOTalon : ClimberIO {
  private val climberTalon: TalonFX = TalonFX(Constants.Climber.CLIMBER_MOTOR_ID)
  private val climberConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val climberSensor =
    ctreAngularMechanismSensor(climberTalon, 1.0, ClimberConstants.VOLTAGE_COMPENSATION)

  private val motionMagicConfiguration = climberConfiguration.MotionMagic
  private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage((-1337).degrees.inDegrees)
  private var slot0Configs = climberConfiguration.Slot0

  private var statorCurrentSignal: StatusSignal<Current>
  private var supplyCurrentSignal: StatusSignal<Current>
  private var tempSignal: StatusSignal<Temperature>
  private var dutyCycle: StatusSignal<Double>
  private var motorVoltage: StatusSignal<Voltage>
  private var motorTorque: StatusSignal<Current>
  private var motorPosition: StatusSignal<edu.wpi.first.units.measure.Angle>
  private var motorVelocity: StatusSignal<AngularVelocity>
  private var motorAcceleration: StatusSignal<AngularAcceleration>

  init {
    climberTalon.configurator.apply(TalonFXConfiguration())
    climberTalon.clearStickyFaults()

    climberConfiguration.Slot0.kP =
      climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_REAL)
    climberConfiguration.Slot0.kI =
      climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_REAL)
    climberConfiguration.Slot0.kD =
      climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_REAL)
    climberConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine

    climberConfiguration.CurrentLimits.SupplyCurrentLimit =
      ClimberConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    climberConfiguration.CurrentLimits.SupplyCurrentLowerLimit =
      ClimberConstants.THRESHOLD_CURRENT_LIMIT.inAmperes
    climberConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    climberConfiguration.CurrentLimits.StatorCurrentLimit =
      ClimberConstants.STATOR_CURRENT_LIMIT.inAmperes
    climberConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

    climberConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
    climberConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true

    climberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    climberConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
    climberTalon.configurator.apply(climberConfiguration)

    statorCurrentSignal = climberTalon.statorCurrent
    supplyCurrentSignal = climberTalon.supplyCurrent
    dutyCycle = climberTalon.dutyCycle
    tempSignal = climberTalon.deviceTemp
    motorVoltage = climberTalon.motorVoltage
    motorTorque = climberTalon.torqueCurrent
    motorPosition = climberTalon.position
    motorVelocity = climberTalon.velocity
    motorAcceleration = climberTalon.acceleration
  }

  override fun updateInputs(inputs: ClimberIO.ClimberInputs) {
    updateSignals()

    climberTalon.rotorPosition.refresh()
    climberTalon.position.refresh()

    inputs.climberPosition = climberSensor.position
    inputs.climberVelocity = climberSensor.velocity
    inputs.climberAcceleration = motorAcceleration.valueAsDouble.degrees.perSecond.perSecond
    inputs.climberTorque = motorTorque.valueAsDouble.newtons
    inputs.climberAppliedVoltage = motorVoltage.valueAsDouble.volts
    inputs.climberDutyCycle = dutyCycle.valueAsDouble.volts
    inputs.climberStatorCurrent = statorCurrentSignal.valueAsDouble.amps
    inputs.climberSupplyCurrent = supplyCurrentSignal.valueAsDouble.amps
    inputs.climberTemperature = tempSignal.valueAsDouble.celsius
  }

  override fun zeroEncoder() {
    climberTalon.setPosition(0.0)
  }

  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    val climberPIDConfig = Slot0Configs()
    climberPIDConfig.kP = climberSensor.proportionalPositionGainToRawUnits(kP)
    climberPIDConfig.kI = climberSensor.integralPositionGainToRawUnits(kI)
    climberPIDConfig.kD = climberSensor.derivativePositionGainToRawUnits(kD)
    climberTalon.configurator.apply(climberPIDConfig)
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    climberTalon.setControl(VoltageOut(voltage.inVolts))
  }

  override fun configFF(
    kG: ElectricalPotential,
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Radian, Volt>,
    kA: AccelerationFeedforward<Radian, Volt>
  ) {
    slot0Configs.kG = kG.inVolts
    slot0Configs.kS = kS.inVolts
    slot0Configs.kV = climberSensor.velocityFeedforwardToRawUnits(kV)
    slot0Configs.kA = climberSensor.accelerationFeedforwardToRawUnits(kA)
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static

    climberTalon.configurator.apply(slot0Configs)
  }

  override fun setPosition(position: Angle, feedforward: ElectricalPotential) {

    climberTalon.setControl(
      motionMagicControl
        .withPosition(climberSensor.positionToRawUnits(position))
        .withFeedForward(feedforward.inVolts)
        .withLimitForwardMotion(true)
        .withLimitReverseMotion(true)
    )
  }

  override fun setBrakeMode(brake: Boolean) {
    val motorOutputConfig = MotorOutputConfigs()

    if (brake) {
      motorOutputConfig.NeutralMode = NeutralModeValue.Brake
    } else {
      motorOutputConfig.NeutralMode = NeutralModeValue.Coast
    }

    climberTalon.configurator.apply(motorOutputConfig)
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
      motorPosition,
      motorVelocity,
      motorAcceleration,
      motorTorque,
      motorVoltage,
      dutyCycle,
      statorCurrentSignal,
      supplyCurrentSignal,
      tempSignal,
    )
  }
}
