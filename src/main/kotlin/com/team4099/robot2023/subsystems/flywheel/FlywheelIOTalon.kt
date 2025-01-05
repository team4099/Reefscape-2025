package com.team4099.robot2023.subsystems.flywheel

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.phoenix6.VelocityVoltage
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.Constants.Shooter.FLYWHEEL_LEFT_MOTOR_ID
import com.team4099.robot2023.config.constants.Constants.Shooter.FLYWHEEL_RIGHT_MOTOR_ID
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.falconspin.Falcon500
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.util.CustomLogger
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.newtons
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object FlywheelIOTalon : FlywheelIO {

  private val flywheelLeftTalon: TalonFX = TalonFX(FLYWHEEL_LEFT_MOTOR_ID)

  private val flywheelRightTalon: TalonFX = TalonFX(FLYWHEEL_RIGHT_MOTOR_ID)

  private val flywheelLeftConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val flywheelRightConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  var velocityRequest =
    VelocityVoltage(-1337.degrees.perSecond, slot = 0, feedforward = -1337.volts)

  private val flywheelLeftSensor =
    ctreAngularMechanismSensor(
      flywheelLeftTalon,
      FlywheelConstants.LEFT_GEAR_RATIO,
      FlywheelConstants.VOLTAGE_COMPENSATION,
    )

  private val flywheelRightSensor =
    ctreAngularMechanismSensor(
      flywheelRightTalon,
      FlywheelConstants.RIGHT_MOTOR_REVOLUTIONS_PER_FLYWHEEL_REVOLUTIONS,
      FlywheelConstants.VOLTAGE_COMPENSATION,
    )

  var leftFlywheelStatorCurrentSignal: StatusSignal<Double>
  var leftFlywheelSupplyCurrentSignal: StatusSignal<Double>
  var leftFlywheelTempSignal: StatusSignal<Double>
  var leftFlywheelDutyCycle: StatusSignal<Double>
  var leftFlywheelAppliedVoltage: StatusSignal<Double>

  var rightFlywheelStatorCurrentSignal: StatusSignal<Double>
  var rightFlywheelSupplyCurrentSignal: StatusSignal<Double>
  var rightFlywheelTempSignal: StatusSignal<Double>
  var rightFlywheelDutyCycle: StatusSignal<Double>
  var motorVoltage: StatusSignal<Double>
  var motorTorque: StatusSignal<Double>
  var leftWheelClosedLoopTarget: StatusSignal<Double>

  init {

    flywheelLeftTalon.configurator.apply(TalonFXConfiguration())
    flywheelRightTalon.configurator.apply(TalonFXConfiguration())

    flywheelLeftTalon.clearStickyFaults()
    flywheelRightTalon.clearStickyFaults()

    flywheelLeftConfiguration.Slot0.kP = 0.4
    flywheelLeftConfiguration.Slot0.kI =
      flywheelLeftSensor.integralVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KI)
    flywheelLeftConfiguration.Slot0.kD =
      flywheelLeftSensor.derivativeVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KD)
    flywheelLeftConfiguration.Slot0.kV =
      flywheelLeftSensor.velocityFeedforwardToRawUnits(FlywheelConstants.PID.REAL_FLYWHEEL_KV)
    flywheelLeftConfiguration.Slot0.kA = FlywheelConstants.PID.REAL_FLYWHEEL_KA.value
    //
    // flywheelSensor.velocityFeedforwardToRawUnits(FlywheelConstantsConstants.PID.flywheel_KFF)

    flywheelLeftConfiguration.CurrentLimits.SupplyCurrentLimit =
      FlywheelConstants.LEFT_FLYWHEEL_SUPPLY_CURRENT_LIMIT.inAmperes
    flywheelLeftConfiguration.CurrentLimits.SupplyCurrentThreshold =
      FlywheelConstants.LEFT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT.inAmperes
    flywheelLeftConfiguration.CurrentLimits.SupplyTimeThreshold =
      FlywheelConstants.LEFT_flywheel_TRIGGER_THRESHOLD_TIME.inSeconds
    flywheelLeftConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    flywheelLeftConfiguration.CurrentLimits.StatorCurrentLimit =
      FlywheelConstants.LEFT_FLYWHEEL_STATOR_CURRENT_LIMIT.inAmperes
    flywheelLeftConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

    flywheelLeftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    //    flywheelRightConfiguration.CurrentLimits.SupplyCurrentLimit =
    //      FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT.inAmperes
    //    flywheelRightConfiguration.CurrentLimits.SupplyCurrentThreshold =
    //      FlywheelConstants.RIGHT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT.inAmperes
    //    flywheelRightConfiguration.CurrentLimits.SupplyTimeThreshold =
    //      FlywheelConstants.RIGHT_flywheel_TRIGGER_THRESHOLD_TIME.inSeconds
    //    flywheelRightConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    //    flywheelRightConfiguration.CurrentLimits.StatorCurrentLimit =
    //      FlywheelConstants.RIGHT_FLYWHEEL_STATOR_CURRENT_LIMIT.inAmperes
    //    flywheelRightConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

    flywheelLeftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
    flywheelRightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast

    flywheelRightTalon.configurator.apply(flywheelRightConfiguration)
    flywheelLeftTalon.configurator.apply(flywheelLeftConfiguration)

    flywheelRightTalon.setControl(Follower(Constants.Shooter.FLYWHEEL_LEFT_MOTOR_ID, true))

    rightFlywheelStatorCurrentSignal = flywheelRightTalon.statorCurrent
    rightFlywheelSupplyCurrentSignal = flywheelRightTalon.supplyCurrent
    rightFlywheelTempSignal = flywheelRightTalon.deviceTemp
    rightFlywheelDutyCycle = flywheelRightTalon.dutyCycle
    motorVoltage = flywheelRightTalon.motorVoltage
    motorTorque = flywheelRightTalon.torqueCurrent

    leftWheelClosedLoopTarget = flywheelLeftTalon.closedLoopReferenceSlope
    leftWheelClosedLoopTarget.setUpdateFrequency(1000.0)

    leftFlywheelStatorCurrentSignal = flywheelLeftTalon.statorCurrent
    leftFlywheelSupplyCurrentSignal = flywheelLeftTalon.supplyCurrent
    leftFlywheelTempSignal = flywheelLeftTalon.deviceTemp
    leftFlywheelDutyCycle = flywheelLeftTalon.dutyCycle
    leftFlywheelAppliedVoltage = flywheelLeftTalon.motorVoltage

    MotorChecker.add(
      "Shooter",
      "Flywheel",
      MotorCollection(
        mutableListOf(Falcon500(flywheelRightTalon, "Flywheel Right Motor")),
        FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT,
        90.celsius,
        FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,
        110.celsius
      )
    )
    MotorChecker.add(
      "Shooter",
      "Flywheel",
      MotorCollection(
        mutableListOf(Falcon500(flywheelLeftTalon, "Flywheel Left Motor")),
        FlywheelConstants.LEFT_FLYWHEEL_SUPPLY_CURRENT_LIMIT,
        90.celsius,
        FlywheelConstants.LEFT_FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,
        110.celsius
      )
    )
  }

  override fun configPID(
    kP: ProportionalGain<Velocity<Radian>, Volt>,
    kI: IntegralGain<Velocity<Radian>, Volt>,
    kD: DerivativeGain<Velocity<Radian>, Volt>,
    kV: VelocityFeedforward<Radian, Volt>
  ) {
    val PIDLeftConfig = Slot0Configs()
    PIDLeftConfig.kP = 0.4
    PIDLeftConfig.kI = flywheelLeftSensor.integralVelocityGainToRawUnits(kI)
    PIDLeftConfig.kD = flywheelLeftSensor.derivativeVelocityGainToRawUnits(kD)
    PIDLeftConfig.kV = flywheelLeftSensor.velocityFeedforwardToRawUnits(kV)

    PIDLeftConfig.kA = FlywheelConstants.PID.REAL_FLYWHEEL_KA.value

    flywheelLeftTalon.configurator.apply(PIDLeftConfig)
  }

  override fun setFlywheelVoltage(voltage: ElectricalPotential) {
    flywheelLeftTalon.setControl(VoltageOut(voltage.inVolts))
  }

  override fun setFlywheelVelocity(velocity: AngularVelocity, feedforward: ElectricalPotential) {
    val error = velocity - flywheelLeftSensor.velocity

    CustomLogger.recordDebugOutput(
      "Flywheel/premotionTargetedVelocity", velocity.inRotationsPerMinute
    )
    CustomLogger.recordDebugOutput(
      "Flywheel/motionTargetedVelocity", flywheelLeftSensor.velocityToRawUnits(velocity)
    )
    //    flywheelRightTalon.setControl(
    //      VelocityVoltage(
    //        flywheelRightSensor.velocityToRawUnits(velocity),
    //        flywheelRightSensor.accelerationToRawUnits(0.0.radians.perSecond.perSecond),
    //        true,
    //        feedforward.inVolts,
    //        0,1
    //        false,1
    //        false,
    //        false
    //      )
    //    )

    var acceleration =
      flywheelLeftSensor.accelerationToRawUnits(20000.rotations.perSecond.perSecond)

    if (velocity.absoluteValue < 50.rotations.perMinute) {
      flywheelLeftTalon.setControl(VoltageOut(0.0.volts.inVolts))
    } else {
      flywheelLeftTalon.setControl(
        MotionMagicVelocityVoltage(
          flywheelLeftSensor.velocityToRawUnits(velocity),
          acceleration,
          false,
          0.0.volts.inVolts,
          0,
          true,
          false,
          false
        )
      )
    }
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
      motorVoltage,
      rightFlywheelStatorCurrentSignal,
      rightFlywheelSupplyCurrentSignal,
      rightFlywheelTempSignal,
      motorTorque,
      rightFlywheelTempSignal,
      leftFlywheelDutyCycle,
      leftFlywheelStatorCurrentSignal,
      leftFlywheelSupplyCurrentSignal,
      leftFlywheelTempSignal,
      leftFlywheelAppliedVoltage
    )
  }

  override fun updateInputs(inputs: FlywheelIO.FlywheelIOInputs) {

    updateSignals()

    inputs.isSimulated = false
    inputs.rightFlywheelVelocity = flywheelRightSensor.velocity
    inputs.rightFlywheelAppliedVoltage = motorVoltage.value.volts
    inputs.rightFlywheelStatorCurrent = rightFlywheelStatorCurrentSignal.value.amps
    inputs.rightFlywheelSupplyCurrent = rightFlywheelSupplyCurrentSignal.value.amps
    inputs.rightFlywheelTemperature = rightFlywheelTempSignal.value.celsius
    // TODO fix unit for torque
    inputs.rightFlywheelTorque = motorTorque.value.newtons
    inputs.rightFlywheelDutyCycle = rightFlywheelDutyCycle.value.volts

    inputs.leftFlywheelVelocity = flywheelLeftSensor.velocity
    inputs.leftFlywheelAppliedVoltage = leftFlywheelAppliedVoltage.value.volts
    inputs.leftFlywheelStatorCurrent = leftFlywheelStatorCurrentSignal.value.amps
    inputs.leftFlywheelSupplyCurrent = leftFlywheelSupplyCurrentSignal.value.amps
    inputs.leftFlywheelTemperature = leftFlywheelTempSignal.value.celsius
    inputs.leftFlywheelTorque = motorTorque.value.newtons

    leftWheelClosedLoopTarget.refresh()
    Logger.recordOutput("Flywheel/profile", leftWheelClosedLoopTarget.value)
  }

  override fun setFlywheelBrakeMode(brake: Boolean) {
    val motorOutputConfig = MotorOutputConfigs()

    if (brake) {
      motorOutputConfig.NeutralMode = NeutralModeValue.Brake
    } else {
      motorOutputConfig.NeutralMode = NeutralModeValue.Coast
    }

    flywheelLeftTalon.configurator.apply(motorOutputConfig)
  }
}
