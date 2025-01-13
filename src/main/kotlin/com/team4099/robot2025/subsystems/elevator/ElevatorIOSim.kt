package com.team4099.robot2025.subsystems.elevator

import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.ElevatorConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import org.team4099.lib.controller.ElevatorFeedforward
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.Value
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

  private var elevatorPIDControllerFirstStage =
    ProfiledPIDController(
      ElevatorConstants.PID.SIM_KP_FIRST_STAGE,
      ElevatorConstants.PID.SIM_KI_FIRST_STAGE,
      ElevatorConstants.PID.SIM_KD_FIRST_STAGE,
      TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION)
    )

  private val elevatorPIDControllerSecondStage =
    ProfiledPIDController(
      ElevatorConstants.PID.SIM_KP_SECOND_STAGE,
      ElevatorConstants.PID.SIM_KI_SECOND_STAGE,
      ElevatorConstants.PID.SIM_KD_SECOND_STAGE,
      TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION)
    )

  private val elevatorPIDControllerThirdStage =
    ProfiledPIDController(
      ElevatorConstants.PID.SIM_KP_THIRD_STAGE,
      ElevatorConstants.PID.SIM_KI_THIRD_STAGE,
      ElevatorConstants.PID.SIM_KD_THIRD_STAGE,
      TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION)
    )

  private var elevatorPIDController = elevatorPIDControllerFirstStage

  private val elevatorFFControllerFirstStage = ElevatorFeedforward(
    ElevatorConstants.PID.SIM_KS_FIRST_STAGE,
    ElevatorConstants.PID.KG_FIRST_STAGE,
    ElevatorConstants.PID.KV_FIRST_STAGE,
    ElevatorConstants.PID.KA_FIRST_STAGE
  )

  private val elevatorFFControllerSecondStage = ElevatorFeedforward(
    ElevatorConstants.PID.SIM_KS_SECOND_STAGE,
    ElevatorConstants.PID.KG_SECOND_STAGE,
    ElevatorConstants.PID.KV_SECOND_STAGE,
    ElevatorConstants.PID.KA_SECOND_STAGE
  )

  private val elevatorFFControllerThirdStage = ElevatorFeedforward(
    ElevatorConstants.PID.SIM_KS_THIRD_STAGE,
    ElevatorConstants.PID.KG_THIRD_STAGE,
    ElevatorConstants.PID.KV_THIRD_STAGE,
    ElevatorConstants.PID.KA_THIRD_STAGE
  )

  private var elevatorFFController = elevatorFFControllerFirstStage;

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
    if (elevatorSim.positionMeters < ElevatorConstants.FIRST_STAGE_HEIGHT.inMeters) {
      elevatorPIDController = elevatorPIDControllerFirstStage
      elevatorFFController = elevatorFFControllerFirstStage
    } else if (elevatorSim.positionMeters < ElevatorConstants.SECOND_STAGE_HEIGHT.inMeters) {
      elevatorPIDController = elevatorPIDControllerSecondStage
      elevatorFFController = elevatorFFControllerSecondStage
    } else {
      elevatorPIDController = elevatorPIDControllerThirdStage
      elevatorFFController = elevatorFFControllerThirdStage
    }

    elevatorPIDController.setGoal(position)

    val pidOutput = elevatorPIDController.calculate(elevatorSim.positionMeters.meters)
    val feedforwardOutput = elevatorFFController.calculate(Value(elevatorPIDController.wpiPidController.getSetpoint().velocity))
    setVoltage(pidOutput + feedforwardOutput)
  }

  override fun zeroEncoder() {}

  /**
   * Updates the PID controller for the first stage of the elevator
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
    elevatorPIDControllerFirstStage.setPID(kP, kI, kD)
  }

  /**
   * Updates the PID controller for the second stage of the elevator
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
    elevatorPIDControllerSecondStage.setPID(kP, kI, kD)
  }

  /**
   * Updates the PID controller for the third stage of the elevator
   *
   * @param kP a constant which will be used to scale the proportion gain
   * @param kI a constant which will be used to scale the integral gain
   * @param kD a constant which will be used to scale the derivative gain
   */
  override fun configThirdStagePID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    elevatorPIDControllerThirdStage.setPID(kP, kI, kD)
  }
}
