package com.team4099.robot2023.subsystems.intake

import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute

object IntakeIOSim : IntakeIO {
  private val rollerSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getNEO(1), IntakeConstants.ROLLER_GEAR_RATIO, IntakeConstants.ROLLER_INERTIA
    )

  private val centerWheelSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getNEO(1),
      IntakeConstants.CENTER_WHEEL_GEAR_RATIO,
      IntakeConstants.CENTER_WHEEL_INERTIA
    )

  private var rollerAppliedVoltage = 0.volts
  private var centerWheelAppliedVoltage = 0.volts
  init {}

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {

    rollerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.rollerVelocity = rollerSim.angularVelocityRPM.rotations.perMinute
    inputs.rollerAppliedVoltage = rollerAppliedVoltage
    inputs.rollerSupplyCurrent = 0.amps
    inputs.rollerStatorCurrent = rollerSim.currentDrawAmps.amps
    inputs.rollerTemp = 0.0.celsius

    inputs.centerWheelVelocity = centerWheelSim.angularVelocityRPM.rotations.perMinute
    inputs.centerWheelAppliedVotlage = centerWheelAppliedVoltage
    inputs.centerWheelSupplyCurrent = 0.amps
    inputs.centerWheelStatorCurrent = centerWheelSim.currentDrawAmps.amps
    inputs.centerWheelTemp = 0.celsius

    inputs.isSimulated = false
  }

  override fun setVoltage(
    rollerVoltage: ElectricalPotential,
    centerWheelVoltage: ElectricalPotential
  ) {
    rollerAppliedVoltage = rollerVoltage
    centerWheelAppliedVoltage = centerWheelVoltage
    rollerSim.setInputVoltage(
      clamp(
        rollerVoltage,
        -IntakeConstants.VOLTAGE_COMPENSATION,
        IntakeConstants.VOLTAGE_COMPENSATION
      )
        .inVolts
    )
    centerWheelSim.setInputVoltage(
      clamp(
        rollerVoltage,
        -IntakeConstants.VOLTAGE_COMPENSATION,
        IntakeConstants.VOLTAGE_COMPENSATION
      )
        .inVolts
    )
  }

  override fun setBrakeMode(rollerBrake: Boolean, centerWheelBrake: Boolean) {}
}
