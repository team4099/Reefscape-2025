package com.team4099.robot2025.subsystems.arm

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.ArmConstants.ARM_KA
import com.team4099.robot2025.config.constants.ArmConstants.ARM_KV
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.arm.ArmIO.ArmIOInputs
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.units.measure.AngularVelocity
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.AngularMechanismSensor
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
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerDegreesPerSecondPerSecond
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.perSecond
import edu.wpi.first.units.measure.AngularAcceleration as WPILibAngularAcceleration
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object ArmIOTalonFX : ArmIO {
  private val armTalon: TalonFX = TalonFX(Constants.Arm.ARM_MOTOR_ID)
  private val armConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  var armAccelerationStatusSignal: StatusSignal<WPILibAngularAcceleration>
  var armTempStatusSignal: StatusSignal<WPILibTemperature>
  var armAppliedVoltageStatusSignal: StatusSignal<WPILibVoltage>
  var armStatorCurrentStatusSignal: StatusSignal<WPILibCurrent>
  var armSupplyCurrentStatusSignal: StatusSignal<WPILibCurrent>

  private var armVelocitySignal: StatusSignal<AngularVelocity>


  val voltageControl: VoltageOut = VoltageOut(0.volts.inVolts)


  init {

    // Configure Current Limits
    armConfiguration.CurrentLimits.StatorCurrentLimit = ArmConstants.STATOR_CURRENT_LIMIT.inAmperes
    armConfiguration.CurrentLimits.SupplyCurrentLimit = ArmConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    armConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
    armConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true

    // Configure Motor Inversion and Break-mode
    armConfiguration.MotorOutput.Inverted = ArmConstants.INVERSION_VALUE
    armConfiguration.MotorOutput.NeutralMode = ArmConstants.NEUTRAL_MODE_VALUE

    // Apply Configs+

    armTalon.configurator.apply(armConfiguration)

    armVelocitySignal = armTalon.velocity
    armAccelerationStatusSignal = armTalon.acceleration
    armTempStatusSignal = armTalon.deviceTemp
    armAppliedVoltageStatusSignal = armTalon.motorVoltage
    armStatorCurrentStatusSignal = armTalon.statorCurrent
    armSupplyCurrentStatusSignal = armTalon.supplyCurrent
  }

  fun updateStatusSignals() {
    BaseStatusSignal.refreshAll(
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


    inputs.armAcceleration = armAccelerationStatusSignal.valueAsDouble.degrees.perSecond.perSecond
    inputs.armAppliedVoltage = armAppliedVoltageStatusSignal.valueAsDouble.volts
    inputs.armStatorCurrent = armStatorCurrentStatusSignal.valueAsDouble.amps
    inputs.armSupplyCurrent = armSupplyCurrentStatusSignal.valueAsDouble.amps
    inputs.armTemp = armTempStatusSignal.valueAsDouble.celsius

  }

  override fun setArmVoltage(voltage: ElectricalPotential) {
    armTalon.setControl(voltageControl.withOutput(voltage.inVolts))
  }


  override fun setArmBrakeMode(brake: Boolean) {
    if (brake) {
      armConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    } else {
      armConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
    }

    armTalon.configurator.apply(armConfiguration)
  }


}
