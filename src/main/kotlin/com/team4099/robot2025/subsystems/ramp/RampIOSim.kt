package com.team4099.robot2025.subsystems.rollers

import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.RampConstants
import com.team4099.robot2025.config.constants.RollersConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute

object RampIOSim : RampIO {

  private val rollerSim: FlywheelSim =
    FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60(1),
        RollersConstants.INERTIA.inKilogramsMeterSquared,
        1 / RollersConstants.GEAR_RATIO
      ),
      DCMotor.getKrakenX60(1),
      1 / RollersConstants.GEAR_RATIO
    )

  private var appliedVoltage = 0.0.volts

  override fun updateInputs(inputs: RampIO.RampIOInputs) {

    rollerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.velocity = rollerSim.angularVelocityRPM.rotations.perMinute
    inputs.appliedVoltage = appliedVoltage
    inputs.statorCurrent = rollerSim.currentDrawAmps.amps
    inputs.supplyCurrent = 0.0.amps
    inputs.motorTemp = 25.0.celsius
    inputs.isSimulating = true
  }

  override fun setVoltage(voltage: ElectricalPotential) {

    val clampedVoltage =
      clamp(voltage, -RampConstants.VOLTAGE_COMPENSATION, RampConstants.VOLTAGE_COMPENSATION)
    rollerSim.setInputVoltage(clampedVoltage.inVolts)
  }

  override fun setBrakeMode(brake: Boolean) {}
}
