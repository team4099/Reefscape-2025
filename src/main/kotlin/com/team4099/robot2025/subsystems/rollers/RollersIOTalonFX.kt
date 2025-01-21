package com.team4099.robot2025.subsystems.rollers

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.RollersConstants
import com.team4099.robot2025.subsystems.rollers.RollersIO.RollersIOInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object RollersIOTalonFX : RollersIO {

  private val rollersTalon: TalonFX = TalonFX(Constants.Rollers.ROLLERS_MOTOR_ID)
  private val rollersConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val rollerSensor =
    ctreAngularMechanismSensor(
      rollersTalon, RollersConstants.GEAR_RATIO, RollersConstants.VOLTAGE_COMPENSATION
    )

  var rollerAppliedVoltageStatusSignal: StatusSignal<WPILibVoltage>
  var rollerStatorCurrentStatusSignal: StatusSignal<WPILibCurrent>
  var rollerSupplyCurrentStatusSignal: StatusSignal<WPILibCurrent>
  var rollerTempStatusSignal: StatusSignal<WPILibTemperature>

  val voltageControl: VoltageOut = VoltageOut(0.volts.inVolts)

  init {

    // configurations
    rollersConfiguration.CurrentLimits.StatorCurrentLimit =
      RollersConstants.STATOR_CURRENT_LIMIT.inAmperes
    rollersConfiguration.CurrentLimits.SupplyCurrentLimit =
      RollersConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    rollersConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
    rollersConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    rollersConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
    rollersConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast

    rollersTalon.configurator.apply(rollersConfiguration)

    // sensor data
    rollerAppliedVoltageStatusSignal = rollersTalon.motorVoltage
    rollerStatorCurrentStatusSignal = rollersTalon.statorCurrent
    rollerSupplyCurrentStatusSignal = rollersTalon.supplyCurrent
    rollerTempStatusSignal = rollersTalon.deviceTemp
  }

  fun refreshStatusSignals() {
    BaseStatusSignal.refreshAll(
      rollerAppliedVoltageStatusSignal,
      rollerStatorCurrentStatusSignal,
      rollerSupplyCurrentStatusSignal,
      rollerTempStatusSignal
    )
  }

  override fun updateInputs(inputs: RollersIOInputs) {
    refreshStatusSignals()
    inputs.rollerVelocity = rollerSensor.velocity
    inputs.rollerAppliedVoltage = rollerAppliedVoltageStatusSignal.valueAsDouble.volts
    inputs.rollerStatorCurrent = rollerStatorCurrentStatusSignal.valueAsDouble.amps
    inputs.rollerSupplyCurrent = rollerSupplyCurrentStatusSignal.valueAsDouble.amps
    inputs.rollerTemp = rollerTempStatusSignal.valueAsDouble.celsius
  }

  override fun setRollerVoltage(voltage: ElectricalPotential) {
    rollersTalon.setControl(voltageControl.withOutput(voltage.inVolts))
  }

  override fun setRollerBrakeMode(brake: Boolean) {
    if (brake) {
      rollersConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    } else {
      rollersConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
    }

    rollersTalon.configurator.apply(rollersConfiguration)
  }
}
