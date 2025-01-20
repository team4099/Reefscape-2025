package com.team4099.robot2025.subsystems.elevator

import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.ElevatorConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inKilograms
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ElevatorIOSim : ElevatorIO {
  private val elevatorSim: ElevatorSim =
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

  private val elevatorPIDController =
    ProfiledPIDController(
      ElevatorConstants.PID.SIM_KP,
      ElevatorConstants.PID.SIM_KI,
      ElevatorConstants.PID.SIM_KD,
      TrapezoidProfile.Constraints(
        ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION
      )
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

  override fun setPosition(position: Length) {
    elevatorPIDController.setGoal(position)
    val pidOutput = elevatorPIDController.calculate(elevatorSim.positionMeters.meters)
    setVoltage(pidOutput)
  }

  override fun zeroEncoder() {}

  /**
   * Updates the PID controller
   *
   * @param kP a constant which will be used to scale the proportion gain
   * @param kI a constant which will be used to scale the integral gain
   * @param kD a constant which will be used to scale the derivative gain
   */
  override fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    elevatorPIDController.setPID(kP, kI, kD)
  }

  /** no ff in sim */
  override fun configFF(
    kG: ElectricalPotential,
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Meter, Volt>,
    kA: AccelerationFeedforward<Meter, Volt>
  ) {
  }
}
