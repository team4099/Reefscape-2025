package com.team4099.robot2025.subsystems.climber

import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ClimberIOSim : ClimberIO {
  private val climberSim: SingleJointedArmSim =
    SingleJointedArmSim(
      DCMotor.getKrakenX60Foc(1),
      1 / ClimberConstants.GEAR_RATIO,
      ClimberConstants.INERTIA.inKilogramsMeterSquared,
      ClimberConstants.LENGTH.inMeters,
      ClimberConstants.MIN_ANGLE.inRadians,
      ClimberConstants.MAX_ANGLE.inRadians,
      true,
      0.0
    )

  private var climberController =
    PIDController(
      ClimberConstants.PID.KP_SIM, ClimberConstants.PID.KI_SIM, ClimberConstants.PID.KD_SIM
    )

  private var targetPosition = 0.degrees
  private var appliedVoltage = 0.volts

  override fun zeroEncoder() {}
  override fun setBrakeMode(brake: Boolean) {}

  /**
   * Updates the PID slot0 constants using the implementation controller, uses arm sensor to convert
   * from PID constants to motor controller units
   *
   * @param kP accounts for linear error
   * @param kI accounts for integral error
   * @param kD accounts for derivative error
   */
  override fun configPIDSlot0(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    climberController.setPID(kP, kI, kD)
  }

  /**
   * Updates the PID slot1 constants using the implementation controller, uses arm sensor to convert
   * from PID constants to motor controller units
   *
   * @param kP accounts for linear error
   * @param kI accounts for integral error
   * @param kD accounts for derivative error
   */
  override fun configPIDSlot1(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    climberController.setPID(kP, kI, kD)
  }

  /**
   * Sets the climber motor voltage and ensures the voltage is limited to battery voltage
   * compensation
   *
   * @param voltage the voltage to set the roller motor to
   */
  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(
        voltage, -ClimberConstants.VOLTAGE_COMPENSATION, ClimberConstants.VOLTAGE_COMPENSATION
      )

    climberSim.setInputVoltage(clampedVoltage.inVolts)
    appliedVoltage = clampedVoltage
  }

  override fun setPosition(position: Angle, latched: Boolean) {
    targetPosition = position
    setVoltage(climberController.calculate(climberSim.angleRads.radians, position))
  }

  override fun updateInputs(inputs: ClimberIO.ClimberInputs) {
    climberSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    inputs.climberPosition = targetPosition
    inputs.climberVelocity = climberSim.velocityRadPerSec.radians.perSecond
    inputs.climberAppliedVoltage = appliedVoltage
    inputs.climberStatorCurrent = climberSim.currentDrawAmps.amps
    inputs.climberSupplyCurrent = 0.amps
    inputs.climberTemperature = 0.0.celsius
    inputs.isSimulated = true
  }
}
