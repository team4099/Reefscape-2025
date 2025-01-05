package com.team4099.robot2023.subsystems.feeder

import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FeederConstants
import edu.wpi.first.math.system.plant.DCMotor
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

object FeederIOSim : FeederIO {
  private val feederSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getNEO(1),
      FeederConstants.FEEDER_GEAR_RATIO,
      FeederConstants.FEEDER_INERTIA.inKilogramsMeterSquared
    )

  var appliedVoltage = 0.volts

  override fun updateInputs(inputs: FeederIO.FeederIOInputs) {
    feederSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.feederVelocity = feederSim.getAngularVelocityRPM().rotations.perMinute
    inputs.feederAppliedVoltage = appliedVoltage
    inputs.feederSupplyCurrent = 0.amps
    inputs.feederStatorCurrent = feederSim.currentDrawAmps.amps
    inputs.feederTemp = 0.0.celsius

    inputs.isSimulated = true
  }

  override fun setFeederVoltage(voltage: ElectricalPotential) {
    appliedVoltage = voltage
    feederSim.setInputVoltage(
      clamp(voltage, -FeederConstants.VOLTAGE_COMPENSATION, FeederConstants.VOLTAGE_COMPENSATION)
        .inVolts
    )
  }
}
