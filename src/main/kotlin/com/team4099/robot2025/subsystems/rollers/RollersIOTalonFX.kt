package com.team4099.robot2025.subsystems.rollers

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.config.constants.RollersConstant
import com.team4099.robot2025.subsystems.rollers.RollersIO.RollersIOInputs
import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond
import edu.wpi.first.units.measure.AngularVelocity as AngularVelocity
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Current as  WPILibCurrent
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object RollersIOTalonFX: RollersIO {

    private val rollersTalon: TalonFX = TalonFX(Constants.Rollers.ROLLERS_MOTOR_ID)
    private val rollersConfiguration: TalonFXConfiguration = TalonFXConfiguration()

    lateinit var rollerVelocityStatusSignal: StatusSignal<AngularVelocity>
    lateinit var rollerAppliedVoltageStatusSignal: StatusSignal<WPILibVoltage>
    lateinit var rollerStatorCurrentStatusSignal: StatusSignal<WPILibCurrent>
    lateinit var rollerSupplyCurrentStatusSignal: StatusSignal<WPILibCurrent>
    lateinit var rollerTempStatusSignal: StatusSignal<WPILibTemperature>

    val voltageControl: VoltageOut = VoltageOut(-1337.volts.inVolts)

    init{

        // current limits
        rollersConfiguration.CurrentLimits.StatorCurrentLimit = RollersConstant.STATOR_CURRENT_LIMIT.inAmperes
        rollersConfiguration.CurrentLimits.SupplyCurrentLimit = RollersConstant.SUPPLY_CURRENT_LIMIT.inAmperes
        rollersConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
        rollersConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true







        // Motor input Data
        rollerVelocityStatusSignal = rollersTalon.velocity
        rollerAppliedVoltageStatusSignal = rollersTalon.motorVoltage
        rollerStatorCurrentStatusSignal = rollersTalon.statorCurrent
        rollerSupplyCurrentStatusSignal = rollersTalon.supplyCurrent
        rollerTempStatusSignal = rollersTalon.deviceTemp


    }

    fun refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
            rollerVelocityStatusSignal,
            rollerAppliedVoltageStatusSignal,
            rollerStatorCurrentStatusSignal,
            rollerSupplyCurrentStatusSignal,
            rollerTempStatusSignal
        )
    }

    override fun updateInputs(inputs: RollersIOInputs) {
        refreshStatusSignals()
        inputs.rollerVelocity = rollerVelocityStatusSignal.valueAsDouble.degrees.perSecond
        inputs.rollerAppliedVoltage  = rollerAppliedVoltageStatusSignal.valueAsDouble.volts
        inputs.rollerStatorCurrent = rollerStatorCurrentStatusSignal.valueAsDouble.amps
        inputs.rollerSupplyCurrent = rollerSupplyCurrentStatusSignal.valueAsDouble.amps
        inputs.rollerTemp = rollerTempStatusSignal.valueAsDouble.celsius
    }


    override fun setRollerVoltage(voltage: ElectricalPotential) {
        rollersTalon.setControl(
            voltageControl.withOutput(voltage.inVolts))
    }

    override fun setRollerBrakeMode(brake: Boolean) {
        if (brake) {
            rollersConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        }
        else {
            rollersConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
        }

        rollersTalon.configurator.apply(rollersConfiguration)
    }
}