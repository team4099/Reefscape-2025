package com.team4099.robot2025.subsystems.rollers

import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.RollersConstant
import com.team4099.robot2025.subsystems.arm.Rollers
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perMinute
import kotlin.math.min
import kotlin.math.max

object RollersIOSim : RollersIO {


    private val rollerSim: FlywheelSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(1),
            RollersConstant.INERTIA.inKilogramsMeterSquared,
            1 / RollersConstant.GEAR_RATIO
        ),
        DCMotor.getKrakenX60(1),
        1 / RollersConstant.GEAR_RATIO
    )


    private var appliedVoltage = 0.0.volts

    override fun updateInputs(inputs: RollersIO.RollersIOInputs) {

        rollerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)


        inputs.rollerVelocity = rollerSim.angularVelocityRPM.rotations.perMinute
        inputs.rollerAppliedVoltage = appliedVoltage
        inputs.rollerStatorCurrent = rollerSim.currentDrawAmps.amps
        inputs.rollerSupplyCurrent = 0.0.amps
        inputs.rollerTemp = 25.0.celsius
    }

    override fun setRollerVoltage(voltage: ElectricalPotential) {

        val clampedVoltage = max(-RollersConstant.MAX_VOLTAGE, min(RollersConstant.MAX_VOLTAGE, voltage.inVolts))
        appliedVoltage = clampedVoltage.volts
        rollerSim.setInputVoltage(clampedVoltage)
    }

    override fun setRollerBrakeMode(brake: Boolean) {

        if (brake) {
            rollerSim.setInputVoltage(0.0)
        }
    }
}