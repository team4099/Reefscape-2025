package com.team4099.robot2023.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import com.team4099.robot2023.subsystems.falconspin.Falcon500
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.subsystems.falconspin.Neo
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import kotlin.math.absoluteValue

object IntakeIOFalconNEO : IntakeIO {
  private val rollerFalcon = TalonFX(Constants.Intake.ROLLER_MOTOR_ID, "rio")

  private val rollerSensor =
    ctreAngularMechanismSensor(
      rollerFalcon, IntakeConstants.ROLLER_GEAR_RATIO, IntakeConstants.VOLTAGE_COMPENSATION
    )

  private val centerWheelSparkMax =
    CANSparkMax(Constants.Intake.CENTER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val centerWheelSensor =
    sparkMaxAngularMechanismSensor(
      centerWheelSparkMax,
      IntakeConstants.CENTER_WHEEL_GEAR_RATIO,
      IntakeConstants.VOLTAGE_COMPENSATION
    )

  var rollerAppliedVoltageSignal: StatusSignal<Double>
  var rollerStatorCurrentSignal: StatusSignal<Double>
  var rollerSupplyCurrentSignal: StatusSignal<Double>
  var rollerTempSignal: StatusSignal<Double>

  init {
    rollerFalcon.configurator

    var rollerFalconConfiguration = TalonFXConfiguration()

    rollerFalconConfiguration.CurrentLimits.StatorCurrentLimit =
      IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT.inAmperes
    rollerFalconConfiguration.CurrentLimits.SupplyCurrentLimit =
      IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT.inAmperes
    //    rollerFalconConfiguration.CurrentLimits.SupplyTimeThreshold =
    // IntakeConstants.ROLLER_CURRENT_TIME_THRESHOLD.inSeconds
    //    rollerFalconConfiguration.CurrentLimits.SupplyCurrentThreshold =
    // IntakeConstants.ROLLER_SUPPLY_TRIGGER_THRESHOLD.inAmperes
    rollerFalconConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
    rollerFalconConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    rollerFalconConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
    rollerFalconConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast

    rollerFalcon.configurator.apply(rollerFalconConfiguration)

    rollerAppliedVoltageSignal = rollerFalcon.motorVoltage
    rollerStatorCurrentSignal = rollerFalcon.statorCurrent
    rollerSupplyCurrentSignal = rollerFalcon.supplyCurrent
    rollerTempSignal = rollerFalcon.deviceTemp

    MotorChecker.add(
      "Intake",
      "Roller",
      MotorCollection(
        mutableListOf(Falcon500(rollerFalcon, "Roller Motor")),
        IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT,
        60.celsius,
        50.amps,
        120.celsius
      ),
    )

    centerWheelSparkMax.restoreFactoryDefaults()
    centerWheelSparkMax.clearFaults()

    centerWheelSparkMax.enableVoltageCompensation(IntakeConstants.VOLTAGE_COMPENSATION.inVolts)
    // centerWheelSparkMax.setSmartCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT.inAmperes.toInt())
    centerWheelSparkMax.inverted = IntakeConstants.ROLLER_MOTOR_INVERTED

    centerWheelSparkMax.idleMode = CANSparkMax.IdleMode.kCoast

    centerWheelSparkMax.burnFlash()

    MotorChecker.add(
      "Intake",
      "Center Wheel",
      MotorCollection(
        mutableListOf(Neo(centerWheelSparkMax, "Center Wheel Motor")),
        IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT,
        60.celsius,
        50.amps,
        120.celsius
      ),
    )
  }

  fun updateSignals() {
    BaseStatusSignal.refreshAll(
      rollerAppliedVoltageSignal,
      rollerStatorCurrentSignal,
      rollerSupplyCurrentSignal,
      rollerTempSignal
    )
  }

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
    updateSignals()

    inputs.rollerVelocity = rollerSensor.velocity
    inputs.rollerAppliedVoltage = rollerAppliedVoltageSignal.value.volts
    inputs.rollerStatorCurrent = rollerStatorCurrentSignal.value.amps
    inputs.rollerSupplyCurrent = rollerSupplyCurrentSignal.value.amps
    inputs.rollerTemp = rollerTempSignal.value.celsius

    inputs.centerWheelVelocity = centerWheelSensor.velocity

    inputs.centerWheelAppliedVotlage =
      centerWheelSparkMax.busVoltage.volts * centerWheelSparkMax.appliedOutput
    inputs.centerWheelStatorCurrent = centerWheelSparkMax.outputCurrent.amps

    inputs.centerWheelSupplyCurrent =
      inputs.centerWheelStatorCurrent * centerWheelSparkMax.appliedOutput.absoluteValue
    inputs.centerWheelTemp = centerWheelSparkMax.motorTemperature.celsius
  }

  /**
   * Sets the roller motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the roller motor to
   */
  override fun setVoltage(
    rollerVoltage: ElectricalPotential,
    centerWheelVoltage: ElectricalPotential
  ) {
    rollerFalcon.setControl(VoltageOut(rollerVoltage.inVolts))

    //    centerWheelSparkMax.setVoltage(
    //      clamp(
    //        centerWheelVoltage,
    //        -IntakeConstants.VOLTAGE_COMPENSATION,
    //        IntakeConstants.VOLTAGE_COMPENSATION
    //      )
    //        .inVolts
    //    )
  }

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  override fun setBrakeMode(rollerBrake: Boolean, centerWheelBrake: Boolean) {
    var motorOutput = MotorOutputConfigs()
    if (rollerBrake) {
      motorOutput.NeutralMode = NeutralModeValue.Brake
    } else {
      motorOutput.NeutralMode = NeutralModeValue.Coast
    }

    rollerFalcon.configurator.apply(motorOutput)

    if (centerWheelBrake) {
      centerWheelSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    } else {
      centerWheelSparkMax.idleMode = CANSparkMax.IdleMode.kCoast
    }
  }
}
