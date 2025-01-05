package com.team4099.robot2023.subsystems.flywheel

import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FlywheelConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute

object FlywheelIOSim : FlywheelIO {
  private val flywheelRightSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getKrakenX60Foc(2),
      1 / FlywheelConstants.RIGHT_MOTOR_REVOLUTIONS_PER_FLYWHEEL_REVOLUTIONS,
      FlywheelConstants.INERTIA.inKilogramsMeterSquared
    )

  private val flywheelLeftSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getKrakenX60Foc(2),
      1 / FlywheelConstants.LEFT_GEAR_RATIO,
      FlywheelConstants.INERTIA.inKilogramsMeterSquared
    )

  private var appliedVoltage = 0.0.volts

  private val flywheelController =
    PIDController(
      FlywheelConstants.PID.SIM_KP, FlywheelConstants.PID.SIM_KI, FlywheelConstants.PID.SIM_KD
    )

  override fun updateInputs(inputs: FlywheelIO.FlywheelIOInputs) {
    flywheelLeftSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    flywheelRightSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.leftFlywheelVelocity = flywheelLeftSim.getAngularVelocityRPM().rotations.perMinute
    inputs.leftFlywheelAppliedVoltage = appliedVoltage
    inputs.leftFlywheelSupplyCurrent = 0.amps
    inputs.leftFlywheelStatorCurrent = flywheelLeftSim.currentDrawAmps.amps
    inputs.leftFlywheelTemperature = 0.0.celsius

    inputs.rightFlywheelVelocity = flywheelRightSim.getAngularVelocityRPM().rotations.perMinute
    inputs.rightFlywheelAppliedVoltage = appliedVoltage
    inputs.rightFlywheelSupplyCurrent = 0.amps
    inputs.rightFlywheelStatorCurrent = flywheelRightSim.currentDrawAmps.amps
    inputs.rightFlywheelTemperature = 0.0.celsius

    inputs.isSimulated = true
  }

  override fun setFlywheelVoltage(
    voltage: ElectricalPotential,
  ) {
    appliedVoltage = voltage
    flywheelRightSim.setInputVoltage(
      clamp(
        voltage,
        -FlywheelConstants.VOLTAGE_COMPENSATION,
        FlywheelConstants.VOLTAGE_COMPENSATION
      )
        .inVolts
    )
    flywheelLeftSim.setInputVoltage(
      clamp(
        voltage,
        -FlywheelConstants.VOLTAGE_COMPENSATION,
        FlywheelConstants.VOLTAGE_COMPENSATION
      )
        .inVolts
    )
  }

  override fun setFlywheelVelocity(velocity: AngularVelocity, feedforward: ElectricalPotential) {
    val feedback =
      flywheelController.calculate(
        flywheelRightSim.getAngularVelocityRPM().rotations.perMinute, velocity
      )

    setFlywheelVoltage(feedback + feedforward)

    appliedVoltage = feedback + feedforward
  }

  override fun setFlywheelBrakeMode(brake: Boolean) {}

  override fun configPID(
    kP: ProportionalGain<Velocity<Radian>, Volt>,
    kI: IntegralGain<Velocity<Radian>, Volt>,
    kD: DerivativeGain<Velocity<Radian>, Volt>,
    kV: VelocityFeedforward<Radian, Volt>
  ) {
    flywheelController.setPID(kP, kI, kD)
  }
}
