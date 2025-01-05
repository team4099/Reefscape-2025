package com.team4099.robot2023.subsystems.wrist

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.Slot2Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.phoenix6.PositionVoltage
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.WristConstants
import com.team4099.robot2023.subsystems.falconspin.Falcon500
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRotations
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

object WristIOTalon : WristIO {

  private val wristTalon: TalonFX = TalonFX(Constants.WRIST.WRIST_MOTOR_ID)

  private val wristConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val absoluteEncoder: CANcoder = CANcoder(Constants.WRIST.CANCODER_ID)
  private val absoluteEncoderConfiguration: MagnetSensorConfigs = MagnetSensorConfigs()

  var positionRequest = PositionVoltage(-1337.degrees, slot = 0, feedforward = -1337.volts)

  val drivenPulley = TunableNumber("Wrist/drivenPulley", 50.0)
  val drivingPulley = TunableNumber("Wrist/drivingPulley", 32.0)

  val firstStagePosErrSwitchThreshold =
    LoggedTunableValue("Wrist/slot1SwitchThreshold", Pair({ it.inDegrees }, { it.degrees }))
  val secondStagePosErrSwitchThreshold =
    LoggedTunableValue("Wrist/slot2SwitchThreshold", Pair({ it.inDegrees }, { it.degrees }))

  val firstStageVelocitySwitchThreshold =
    LoggedTunableValue(
      "Wrist/slot1VelSwitchThreshold",
      Pair({ it.inDegreesPerSecond }, { it.degrees.perSecond })
    )
  val secondStageVelocitySwitchThreshold =
    LoggedTunableValue(
      "Wrist/slot2VelSwitchThreshold",
      Pair({ it.inDegreesPerSecond }, { it.degrees.perSecond })
    )

  private val wristSensor =
    ctreAngularMechanismSensor(
      wristTalon,
      1.0,
      WristConstants.VOLTAGE_COMPENSATION,
    )

  var statorCurrentSignal: StatusSignal<Double>
  var supplyCurrentSignal: StatusSignal<Double>
  var tempSignal: StatusSignal<Double>
  var dutyCycle: StatusSignal<Double>
  var motorVoltage: StatusSignal<Double>
  var motorTorque: StatusSignal<Double>
  var motorAcceleration: StatusSignal<Double>
  var absoluteEncoderSignal: StatusSignal<Double>

  var angleToZero: Angle = 0.0.degrees

  init {
    firstStagePosErrSwitchThreshold.initDefault(WristConstants.PID.FIRST_STAGE_POS_SWITCH_THRESHOLD)
    secondStagePosErrSwitchThreshold.initDefault(
      WristConstants.PID.SECOND_STAGE_POS_SWITCH_THRESHOLD
    )

    firstStageVelocitySwitchThreshold.initDefault(
      WristConstants.PID.FIRST_STAGE_VEL_SWITCH_THRESHOLD
    )
    secondStageVelocitySwitchThreshold.initDefault(
      WristConstants.PID.SECOND_STAGE_VEL_SWITCH_THRESHOLD
    )

    wristTalon.configurator.apply(TalonFXConfiguration())

    wristTalon.clearStickyFaults()

    absoluteEncoderConfiguration.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf
    absoluteEncoderConfiguration.SensorDirection = SensorDirectionValue.Clockwise_Positive
    absoluteEncoderConfiguration.MagnetOffset = WristConstants.ABSOLUTE_ENCODER_OFFSET.inRotations

    absoluteEncoder.configurator.apply(absoluteEncoderConfiguration)

    wristConfiguration.Feedback.FeedbackRemoteSensorID = absoluteEncoder.deviceID
    wristConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder

    wristConfiguration.Feedback.SensorToMechanismRatio =
      WristConstants.ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO
    wristConfiguration.Feedback.RotorToSensorRatio =
      WristConstants.MOTOR_TO_ABSOLUTE_ENCODER_GEAR_RATIO

    wristConfiguration.Slot0.kP =
      wristSensor.proportionalPositionGainToRawUnits(WristConstants.PID.REAL_KP)
    wristConfiguration.Slot0.kI =
      wristSensor.integralPositionGainToRawUnits(WristConstants.PID.REAL_KI)
    wristConfiguration.Slot0.kD =
      wristSensor.derivativePositionGainToRawUnits(WristConstants.PID.REAL_KD)

    wristConfiguration.Slot1.kP =
      wristSensor.proportionalPositionGainToRawUnits(WristConstants.PID.FIRST_STAGE_KP)
    wristConfiguration.Slot1.kI =
      wristSensor.integralPositionGainToRawUnits(WristConstants.PID.FIRST_STAGE_KI)
    wristConfiguration.Slot1.kD =
      wristSensor.derivativePositionGainToRawUnits(WristConstants.PID.FIRST_STAGE_KD)

    wristConfiguration.Slot2.kP =
      wristSensor.proportionalPositionGainToRawUnits(WristConstants.PID.SECOND_STAGE_KP)
    wristConfiguration.Slot2.kI =
      wristSensor.integralPositionGainToRawUnits(WristConstants.PID.SECOND_STAGE_KI)
    wristConfiguration.Slot2.kD =
      wristSensor.derivativePositionGainToRawUnits(WristConstants.PID.SECOND_STAGE_KD)

    wristConfiguration.Voltage.PeakForwardVoltage = 3.0
    wristConfiguration.Voltage.PeakReverseVoltage = -3.0

    wristConfiguration.CurrentLimits.SupplyCurrentLimit =
      WristConstants.WRIST_SUPPLY_CURRENT_LIMIT.inAmperes
    wristConfiguration.CurrentLimits.SupplyCurrentThreshold =
      WristConstants.WRIST_THRESHOLD_CURRENT_LIMIT.inAmperes
    wristConfiguration.CurrentLimits.SupplyTimeThreshold =
      WristConstants.WRIST_TRIGGER_THRESHOLD_TIME.inSeconds
    wristConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    wristConfiguration.CurrentLimits.StatorCurrentLimit =
      WristConstants.WRIST_STATOR_CURRENT_LIMIT.inAmperes
    wristConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

    wristConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    wristConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

    wristTalon.configurator.apply(wristConfiguration)

    statorCurrentSignal = wristTalon.statorCurrent
    supplyCurrentSignal = wristTalon.supplyCurrent
    tempSignal = wristTalon.deviceTemp
    dutyCycle = wristTalon.dutyCycle
    motorVoltage = wristTalon.motorVoltage
    motorTorque = wristTalon.torqueCurrent
    motorAcceleration = wristTalon.acceleration

    absoluteEncoderSignal = absoluteEncoder.position

    MotorChecker.add(
      "Wrist",
      "Wrist",
      MotorCollection(
        mutableListOf(Falcon500(wristTalon, "Wrist Motor")),
        WristConstants.WRIST_STATOR_CURRENT_LIMIT,
        90.celsius,
        WristConstants.WRIST_STATOR_CURRENT_LIMIT - 30.amps,
        110.celsius
      )
    )
  }

  override fun configPIDSlot2(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    val PIDConfig = Slot2Configs()
    PIDConfig.kP = wristSensor.proportionalPositionGainToRawUnits(kP)
    PIDConfig.kI = wristSensor.integralPositionGainToRawUnits(kI)
    PIDConfig.kD = wristSensor.derivativePositionGainToRawUnits(kD)

    wristTalon.configurator.apply(PIDConfig)
  }

  override fun configPIDSlot1(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    val PIDConfig = Slot1Configs()
    PIDConfig.kP = wristSensor.proportionalPositionGainToRawUnits(kP)
    PIDConfig.kI = wristSensor.integralPositionGainToRawUnits(kI)
    PIDConfig.kD = wristSensor.derivativePositionGainToRawUnits(kD)

    wristTalon.configurator.apply(PIDConfig)
  }

  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    val PIDConfig = Slot0Configs()
    PIDConfig.kP = wristSensor.proportionalPositionGainToRawUnits(kP)
    PIDConfig.kI = wristSensor.integralPositionGainToRawUnits(kI)
    PIDConfig.kD = wristSensor.derivativePositionGainToRawUnits(kD)

    wristTalon.configurator.apply(PIDConfig)
  }

  override fun setWristVoltage(voltage: ElectricalPotential) {
    wristTalon.setControl(VoltageOut(voltage.inVolts))
  }

  override fun setWristPosition(
    position: Angle,
    feedforward: ElectricalPotential,
    travelingUp: Boolean
  ) {
    positionRequest.setFeedforward(feedforward)
    positionRequest.setPosition(position)

    val curError = (wristSensor.position - position)
    val curVel = wristSensor.velocity

    var slot = if (travelingUp) 0 else 1

    if (curError.absoluteValue <= secondStagePosErrSwitchThreshold.get() &&
      curVel.absoluteValue <= secondStageVelocitySwitchThreshold.get()
    ) {
      slot = 2
    }

    Logger.recordOutput("Wrist/feedForwardApplied", feedforward.inVolts)
    Logger.recordOutput("Wrist/slotBeingUsed", slot)

    wristTalon.setControl(
      PositionDutyCycle(
        wristSensor.positionToRawUnits(position),
        wristSensor.velocityToRawUnits(0.0.radians.perSecond),
        true,
        feedforward.inVolts,
        slot,
        false,
        false,
        false
      )
    )
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
      motorTorque,
      motorVoltage,
      dutyCycle,
      supplyCurrentSignal,
      statorCurrentSignal,
      tempSignal,
      motorAcceleration,
      absoluteEncoderSignal
    )
  }

  override fun updateInputs(inputs: WristIO.WristIOInputs) {
    updateSignals()

    wristTalon.rotorPosition.refresh()
    wristTalon.position.refresh()
    Logger.recordOutput(
      "Wrist/rotorTMechanismDegrees",
      (
        (
          wristTalon.rotorPosition.value.rotations /
            WristConstants.MOTOR_TO_ABSOLUTE_ENCODER_GEAR_RATIO /
            (1.06488 / 1.0)
          ) + angleToZero
        )
        .inDegrees
    )
    Logger.recordOutput("Wrist/absoluteEncoderRots", absoluteEncoderSignal.value)
    Logger.recordOutput(
      "Wrist/absoluteEncoderTMechanismDegrees",
      (absoluteEncoderSignal.value / (drivenPulley.get() / drivingPulley.get()))
        .rotations
        .inDegrees
    )
    Logger.recordOutput("Wrist/hopefullyAbsoluteEncoderRots", wristTalon.position.value)

    inputs.wristAbsoluteEncoderPosition =
      (absoluteEncoderSignal.value).rotations /
      WristConstants.ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO
    inputs.wristPosition = wristSensor.position
    //      wristTalon.position.value.rotations /
    //      WristConstants.MOTOR_TO_ABSOLUTE_ENCODER_GEAR_RATIO /
    //      WristConstants.ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO + angleToZero
    inputs.wristAcceleration =
      (motorAcceleration.value * WristConstants.ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO)
        .rotations
        .perSecond
        .perSecond
    inputs.wristVelocity = wristSensor.velocity
    // TODO fix unit for torque
    inputs.wristTorque = motorTorque.value
    inputs.wristAppliedVoltage = motorVoltage.value.volts
    inputs.wristDutyCycle = dutyCycle.value.volts
    inputs.wristStatorCurrent = statorCurrentSignal.value.amps
    inputs.wristSupplyCurrent = supplyCurrentSignal.value.amps
    inputs.wristTemperature = tempSignal.value.celsius

    if (inputs.wristPosition < WristConstants.WRIST_MIN_ROTATION) {
      zeroEncoder()
    }
  }

  override fun setWristBrakeMode(brake: Boolean) {
    val motorOutputConfig = MotorOutputConfigs()

    if (brake) {
      motorOutputConfig.NeutralMode = NeutralModeValue.Brake
    } else {
      motorOutputConfig.NeutralMode = NeutralModeValue.Coast
    }
    wristTalon.configurator.apply(motorOutputConfig)
  }

  override fun zeroEncoder() {
    angleToZero =
      (absoluteEncoderSignal.value).rotations /
      WristConstants.ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO
    Logger.recordOutput("Wrist/angleToZero", angleToZero.inDegrees)
    wristTalon.setPosition(angleToZero.inRotations)
  }
}
