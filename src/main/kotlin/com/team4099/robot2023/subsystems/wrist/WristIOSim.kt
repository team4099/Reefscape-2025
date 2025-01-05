package com.team4099.robot2023.subsystems.wrist

import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.WristConstants
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.subsystems.falconspin.SimulatedMotor
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

object WristIOSim : WristIO {

  val wristSim =
    SingleJointedArmSim(
      DCMotor.getNEO(1),
      1 / WristConstants.ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO,
      WristConstants.WRIST_INERTIA.inKilogramsMeterSquared,
      WristConstants.WRIST_LENGTH.inMeters,
      WristConstants.WRIST_MIN_ROTATION.inRadians,
      WristConstants.WRIST_MAX_ROTATION.inRadians,
      true,
      0.0
    )

  var wristTargetPosition = -35.0.degrees

  init {
    MotorChecker.add(
      "Ground Intake",
      "Rotation",
      MotorCollection(
        mutableListOf(
          SimulatedMotor(
            wristSim,
            "Arm Motor",
          ),
        ),
        60.amps,
        10.celsius,
        45.amps,
        20.celsius
      )
    )

    MotorChecker.add(
      "Ground Intake",
      "Roller",
      MotorCollection(
        mutableListOf(
          SimulatedMotor(
            wristSim,
            "Roller Motor",
          )
        ),
        60.amps,
        10.celsius,
        45.amps,
        20.celsius
      )
    )
  }

  private val wristController =
    PIDController(WristConstants.PID.SIM_KP, WristConstants.PID.SIM_KI, WristConstants.PID.SIM_KD)

  var appliedVoltage = 0.volts

  override fun updateInputs(inputs: WristIO.WristIOInputs) {
    wristSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.wristPosition = wristTargetPosition
    inputs.wristVelocity = wristSim.velocityRadPerSec.radians.perSecond
    inputs.wristSupplyCurrent = 0.amps
    inputs.wristAppliedVoltage = appliedVoltage
    inputs.wristStatorCurrent = wristSim.currentDrawAmps.amps
    inputs.wristTemperature = 0.0.celsius

    inputs.isSimulated = true
  }

  /**
   * Sets the roller motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the roller motor to
   */
  override fun setWristVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(voltage, -WristConstants.VOLTAGE_COMPENSATION, WristConstants.VOLTAGE_COMPENSATION)
    wristSim.setInputVoltage(clampedVoltage.inVolts)
    appliedVoltage = clampedVoltage
  }

  override fun setWristPosition(
    position: Angle,
    feedforward: ElectricalPotential,
    travelingUp: Boolean
  ) {
    wristTargetPosition = position
    val feedback = wristController.calculate(wristSim.angleRads.radians, position)
    setWristVoltage(feedback + feedforward)
  }

  /**
   * Updates the PID constants using the implementation controller, uses arm sensor to convert from
   * PID constants to motor controller units
   *
   * @param kP accounts for linear error
   * @param kI accounts for integral error
   * @param kD accounts for derivative error
   */
  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    wristController.setPID(kP, kI, kD)
  }

  /** recalculates the current position of the neo encoder using value from the absolute encoder */
  override fun zeroEncoder() {}
}
