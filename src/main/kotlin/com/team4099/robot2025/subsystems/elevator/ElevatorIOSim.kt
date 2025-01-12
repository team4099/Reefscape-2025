package com.team4099.robot2025.subsystems.elevator

import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.ElevatorConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inKilograms
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ElevatorIOSim : ElevatorIO {
  val elevatorSim: ElevatorSim =
    ElevatorSim(
      DCMotor.getKrakenX60Foc(2),
      1.0 / ElevatorConstants.GEAR_RATIO,
      ElevatorConstants.CARRIAGE_MASS.inKilograms,
      ElevatorConstants.SPOOL_DIAMETER.inMeters / 2.0,
      ElevatorConstants.DOWNWARDS_EXTENSION_LIMIT.inMeters,
      ElevatorConstants.UPWARDS_EXTENSION_LIMIT.inMeters,
      true,
      0.0
    )

  private var lastAppliedVoltage = 0.0.volts

  private val elevatorController =
    PIDController(
      ElevatorConstants.PID.SIM_KP, ElevatorConstants.PID.SIM_KI, ElevatorConstants.PID.SIM_KD
    )

  override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {
    elevatorSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.elevatorPosition = elevatorSim.positionMeters.meters
    inputs.elevatorVelocity = elevatorSim.velocityMetersPerSecond.meters.perSecond

    inputs.leaderTemperature = 0.0.celsius
    inputs.leaderStatorCurrent = 0.0.amps
    inputs.leaderSupplyCurrent = elevatorSim.currentDrawAmps.amps / 2
    inputs.leaderAppliedVoltage = lastAppliedVoltage

    inputs.followerTemperature = 0.0.celsius
    inputs.followerStatorCurrent = 0.0.amps
    inputs.followerSupplyCurrent = elevatorSim.currentDrawAmps.amps / 2
    inputs.followerAppliedVoltage = lastAppliedVoltage

    inputs.isSimulating = true

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.currentDrawAmps)
    )
  }

  override fun setVoltage(targetVoltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(
        targetVoltage,
        -ElevatorConstants.VOLTAGE_COMPENSATION,
        ElevatorConstants.VOLTAGE_COMPENSATION
      )
    lastAppliedVoltage = clampedVoltage

    elevatorSim.setInputVoltage(clampedVoltage.inVolts)
  }

  /**
   * Sets the voltage of the elevator motors
   *
   * @param position the target position the PID controller will use
   */
  override fun setPosition(position: Length) {
    elevatorSim.setState(position.inMeters, elevatorSim.velocityMetersPerSecond)
  }

  /** set the current encoder position to be the encoders zero value */
  override fun zeroEncoder() {}

  /**
   * updates the PID controller values using the sensor measurement for proportional intregral and
   * derivative gain multiplied by the 3 PID constants for the first stage
   *
   * @param kP a constant which will be used to scale the proportion gain
   * @param kI a constant which will be used to scale the integral gain
   * @param kD a constant which will be used to scale the derivative gain
   */
  override fun configFirstStagePID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    elevatorController.setPID(kP, kI, kD)
  }

  /**
   * updates the PID controller values using the sensor measurement for proportional intregral and
   * derivative gain multiplied by the 3 PID constants for the second stage
   *
   * @param kP a constant which will be used to scale the proportion gain
   * @param kI a constant which will be used to scale the integral gain
   * @param kD a constant which will be used to scale the derivative gain
   */
  override fun configSecondStagePID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    elevatorController.setPID(kP, kI, kD)
  }
}
