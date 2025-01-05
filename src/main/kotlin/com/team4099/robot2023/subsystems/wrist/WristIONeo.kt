package com.team4099.robot2023.subsystems.wrist

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.WristConstants
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import kotlin.math.IEEErem
import kotlin.math.absoluteValue

object WristIONeo : WristIO {
  // private val rollerSparkMax = CANSparkMax(Constants.Shooter.ROLLER_MOTOR_ID,
  // CANSparkMaxLowLevel.MotorType.kBrushless)
  // private val rollerSensor = sparkMaxAngularMechanismSensor( rollerSparkMax,
  // ShooterConstants.ROLLER_GEAR_RATIO,
  // ShooterConstants.ROLLER_VOLTAGE_COMPENSATION
  // )

  private val wristSparkMax =
    CANSparkMax(Constants.WRIST.WRIST_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val wristSensor =
    sparkMaxAngularMechanismSensor(
      wristSparkMax,
      WristConstants.ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO,
      WristConstants.WRIST_VOLTAGE_COMPENSATION
    )
  private val wristPIDController: SparkMaxPIDController = wristSparkMax.pidController
  // private val feederSparkMax = CANSparkMax(Constants.Shooter.FEEDER_MOTOR_ID,
  // CANSparkMaxLowLevel.MotorType.kBrushless)
  // private val feederSensor = sparkMaxAngularMechanismSensor( feederSparkMax,
  // ShooterConstants.FEEDER_GEAR_RATIO,
  // ShooterConstants.FEEDER_VOLTAGE_COMPENSATION
  // )

  private val throughBoreEncoder = DutyCycleEncoder(Constants.Intake.REV_ENCODER_PORT)

  private val encoderAbsolutePosition: Angle
    get() {
      var output =
        (
          (-throughBoreEncoder.absolutePosition.rotations) /
            WristConstants.ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO
          )

      if (output in (-55).degrees..0.0.degrees) {
        output -= 180.degrees
      }

      return output
    }

  // uses the absolute encoder position to calculate the arm position
  private val armAbsolutePosition: Angle
    get() {
      return (encoderAbsolutePosition).inDegrees.IEEErem(360.0).degrees
    }

  init {
    // reset the motors
    // rollerSparkMax.restoreFactoryDefaults()
    // rollerSparkMax.clearFaults()

    wristSparkMax.restoreFactoryDefaults()
    wristSparkMax.clearFaults()

    // feederSparkMax.restoreFactoryDefaults()
    // feederSparkMax.clearFaults()

    // voltage and current limits
    // rollerSparkMax.enableVoltageCompensation(ShooterConstants.ROLLER_VOLTAGE_COMPENSATION.inVolts)
    // rollerSparkMax.setSmartCurrentLimit(ShooterConstants.ROLLER_STATOR_CURRENT_LIMIT.inAmperes.toInt())

    wristSparkMax.enableVoltageCompensation(WristConstants.WRIST_VOLTAGE_COMPENSATION.inVolts)
    wristSparkMax.setSmartCurrentLimit(WristConstants.WRIST_STATOR_CURRENT_LIMIT.inAmperes.toInt())

    // feederSparkMax.enableVoltageCompensation(ShooterConstants.FEEDER_VOLTAGE_COMPENSATION.inVolts)
    // feederSparkMax.setSmartCurrentLimit(ShooterConstants.FEEDER_STATOR_CURRENT_LIMIT.inAmperes.toInt())
  }

  override fun updateInputs(inputs: WristIO.WristIOInputs) {
    /*   inputs.rollerVelocity = rollerSensor.velocity
            inputs.rollerAppliedVoltage = rollerSparkMax.busVoltage.volts * rollerSparkMax.appliedOutput
            inputs.rollerStatorCurrent = rollerSparkMax.outputCurrent.amps
            // BatteryVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
            // AppliedVoltage = percentOutput * BatteryVoltage
            // SupplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
            // percentOutput * statorCurrent
            inputs.rollerSupplyCurrent = inputs.rollerStatorCurrent * rollerSparkMax.appliedOutput.absoluteValue
            inputs.rollerTempreature = rollerSparkMax.motorTemperature.celsius
    */
    inputs.wristPosition = wristSensor.position
    inputs.wristAppliedVoltage = wristSparkMax.busVoltage.volts * wristSparkMax.appliedOutput
    inputs.wristStatorCurrent = wristSparkMax.outputCurrent.amps
    // BatteryVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BatteryVoltage
    // SupplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
    // percentOutput * statorCurrent
    inputs.wristSupplyCurrent =
      inputs.wristStatorCurrent * wristSparkMax.appliedOutput.absoluteValue
    inputs.wristTemperature = wristSparkMax.motorTemperature.celsius

    /* inputs.feederVelocity = feederSensor.velocity
    inputs.feederAppliedVoltage = feederSparkMax.busVoltage.volts * feederSparkMax.appliedOutput
    inputs.feederStatorCurrent = feederSparkMax.outputCurrent.amps
    // BatteryVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BatteryVoltage
    // SupplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
    // percentOutput * statorCurrent
    inputs.feederSupplyCurrent = inputs.wristStatorCurrent * feederSparkMax.appliedOutput.absoluteValue
    inputs.feederTemperature = feederSparkMax.motorTemperature.celsius*/
  }
  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    wristPIDController.p = wristSensor.proportionalPositionGainToRawUnits(kP)
    wristPIDController.i = wristSensor.integralPositionGainToRawUnits(kI)
    wristPIDController.d = wristSensor.derivativePositionGainToRawUnits(kD)
  }

  override fun setWristPosition(
    position: Angle,
    feedforward: ElectricalPotential,
    travelingUp: Boolean
  ) {
    wristPIDController.setReference(
      wristSensor.positionToRawUnits(
        clamp(position, WristConstants.WRIST_MIN_ROTATION, WristConstants.WRIST_MAX_ROTATION)
      ),
      CANSparkMax.ControlType.kPosition,
      0,
      feedforward.inVolts
    )
  }

  override fun setWristVoltage(voltage: ElectricalPotential) {
    wristSparkMax.setVoltage(
      clamp(
        voltage,
        -WristConstants.WRIST_VOLTAGE_COMPENSATION,
        WristConstants.WRIST_VOLTAGE_COMPENSATION
      )
        .inVolts
    )
  }

  override fun setWristBrakeMode(brake: Boolean) {
    if (brake) {
      wristSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)
    } else {
      wristSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast)
    }
  }

  override fun zeroEncoder() {
    wristSparkMax.encoder.position = wristSensor.positionToRawUnits(armAbsolutePosition)
  }
}
