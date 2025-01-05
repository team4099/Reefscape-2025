package com.team4099.robot2023.subsystems.feeder

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxLimitSwitch
import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FeederConstants
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.subsystems.falconspin.Neo
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import kotlin.math.absoluteValue

object FeederIONeo : FeederIO {
  private val feederSparkMax =
    CANSparkMax(Constants.Feeder.FEEDER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val feederSensor =
    sparkMaxAngularMechanismSensor(
      feederSparkMax, FeederConstants.FEEDER_GEAR_RATIO, FeederConstants.VOLTAGE_COMPENSATION
    )

  private val beamBreakPort =
    feederSparkMax.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed)

  init {
    feederSparkMax.restoreFactoryDefaults()
    feederSparkMax.clearFaults()

    feederSparkMax.enableVoltageCompensation(FeederConstants.VOLTAGE_COMPENSATION.inVolts)
    feederSparkMax.setSmartCurrentLimit(FeederConstants.FEEDER_CURRENT_LIMIT.inAmperes.toInt())
    feederSparkMax.inverted = FeederConstants.FEEDER_MOTOR_INVERTED

    feederSparkMax.idleMode = CANSparkMax.IdleMode.kBrake

    beamBreakPort.enableLimitSwitch(false)

    feederSparkMax.burnFlash()

    MotorChecker.add(
      "Feeder",
      "Roller",
      MotorCollection(
        mutableListOf(Neo(feederSparkMax, "Roller Motor")),
        FeederConstants.FEEDER_CURRENT_LIMIT,
        70.celsius,
        FeederConstants.FEEDER_CURRENT_LIMIT - 10.amps,
        90.celsius
      ),
    )
  }

  override fun updateInputs(inputs: FeederIO.FeederIOInputs) {
    inputs.feederVelocity = feederSensor.velocity
    inputs.feederAppliedVoltage = feederSparkMax.busVoltage.volts * feederSparkMax.appliedOutput
    inputs.feederStatorCurrent = feederSparkMax.outputCurrent.amps
    inputs.beamBroken = beamBreakPort.isPressed

    // BusVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BusVoltage
    // SupplyCurrent = (percentOutput * BusVoltage / BusVoltage) * StatorCurrent =
    // percentOutput * statorCurrent
    inputs.feederSupplyCurrent =
      inputs.feederStatorCurrent * feederSparkMax.appliedOutput.absoluteValue
    inputs.feederTemp = feederSparkMax.motorTemperature.celsius
  }

  /**
   * Sets the roller motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the roller motor to
   */
  override fun setFeederVoltage(voltage: ElectricalPotential) {
    feederSparkMax.setVoltage(
      clamp(voltage, -FeederConstants.VOLTAGE_COMPENSATION, FeederConstants.VOLTAGE_COMPENSATION)
        .inVolts
    )
  }

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  override fun setFeederBrakeMode(brake: Boolean) {
    if (brake) {
      feederSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    } else {
      feederSparkMax.idleMode = CANSparkMax.IdleMode.kCoast
    }
  }
}
