package com.team4099.robot2025.subsystems.arm

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.arm.ArmIO.ArmIOInputs
import edu.wpi.first.units.measure.AngularVelocity
import org.team4099.lib.units.AngularMechanismSensor
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.inRadiansPerSecondPerSecond
import org.team4099.lib.units.perSecond
import edu.wpi.first.units.measure.AngularAcceleration as WPILibAngularAcceleration
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object ArmIOTalonFX : ArmIO {
  private val armTalon: TalonFX = TalonFX(Constants.Arm.ARM_MOTOR_ID)
  private val armConfiguration: TalonFXConfiguration = TalonFXConfiguration()
  private val positionControl: MotionMagicVoltage = MotionMagicVoltage(0.degrees.inDegrees)

  val armSensor: AngularMechanismSensor =
    ctreAngularMechanismSensor(
      armTalon,
      ArmConstants.ARM_GEAR_RATIO,
      ArmConstants.VOLTAGE_COMPENSATION
    )

  var armPositionStatusSignal: StatusSignal<edu.wpi.first.units.measure.Angle>
  var armAccelerationStatusSignal: StatusSignal<WPILibAngularAcceleration>
  var armTempStatusSignal: StatusSignal<WPILibTemperature>
  var armAppliedVoltageStatusSignal: StatusSignal<WPILibVoltage>
  var armStatorCurrentStatusSignal: StatusSignal<WPILibCurrent>
  var armSupplyCurrentStatusSignal: StatusSignal<WPILibCurrent>

  private var armVelocitySignal: StatusSignal<AngularVelocity>

  init {
    // Configure Current Limits
    armConfiguration.CurrentLimits.StatorCurrentLimit = ArmConstants.STATOR_CURRENT_LIMIT.inAmperes
    armConfiguration.CurrentLimits.SupplyCurrentLimit = ArmConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    armConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
    armConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true

    // Configure Motor Inversion and Break-mode
    armConfiguration.MotorOutput.Inverted = ArmConstants.INVERSION_VALUE
    armConfiguration.MotorOutput.NeutralMode = ArmConstants.NEUTRAL_MODE_VALUE

    armConfiguration.MotionMagic.MotionMagicAcceleration = ArmConstants.MAX_ACCELERATION.inDegreesPerSecondPerSecond
    armConfiguration.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MAX_VELOCITY.inDegreesPerSecond

    // Apply Configs+

    armTalon.configurator.apply(armConfiguration)

    armPositionStatusSignal = armTalon.position
    armVelocitySignal = armTalon.velocity
    armAccelerationStatusSignal = armTalon.acceleration
    armTempStatusSignal = armTalon.deviceTemp
    armAppliedVoltageStatusSignal = armTalon.motorVoltage
    armStatorCurrentStatusSignal = armTalon.statorCurrent
    armSupplyCurrentStatusSignal = armTalon.supplyCurrent
  }

  fun updateStatusSignals() {
    BaseStatusSignal.refreshAll(
      armPositionStatusSignal,
      armVelocitySignal,
      armAccelerationStatusSignal,
      armTempStatusSignal,
      armAppliedVoltageStatusSignal,
      armStatorCurrentStatusSignal,
      armSupplyCurrentStatusSignal,
    )
  }

  override fun updateInputs(inputs: ArmIOInputs) {
    updateStatusSignals()

    inputs.armPosition = armSensor.position
    inputs.armAcceleration = armAccelerationStatusSignal.valueAsDouble.degrees.perSecond.perSecond
    inputs.armAppliedVoltage = armAppliedVoltageStatusSignal.valueAsDouble.volts
    inputs.armStatorCurrent = armStatorCurrentStatusSignal.valueAsDouble.amps
    inputs.armSupplyCurrent = armSupplyCurrentStatusSignal.valueAsDouble.amps
    inputs.armTemp = armTempStatusSignal.valueAsDouble.celsius
  }

  override fun setArmPosition(position: Angle) {
    armTalon.setControl(positionControl.withPosition(armSensor.positionToRawUnits(position)))
  }

  override fun configurePID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    val PIDConfig = armConfiguration.Slot0

    PIDConfig.kP = kP.inVoltsPerDegree
    PIDConfig.kI = kI.inVoltsPerDegreeSeconds
    PIDConfig.kD = kD.inVoltsPerDegreePerSecond

    PIDConfig.GravityType = GravityTypeValue.Arm_Cosine
    PIDConfig.kG = 0.2

    armTalon.configurator.apply(PIDConfig)
  }

  override fun setArmBrakeMode(brake: Boolean) {
    if (brake) {
      armConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    } else {
      armConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
    }

    armTalon.configurator.apply(armConfiguration)
  }

  override fun zeroEncoder() {
    armTalon.setPosition(0.0)
  }
}
