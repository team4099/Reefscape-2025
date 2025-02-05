package com.team4099.robot2025.subsystems.arm

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.ArmConstants.ARM_KA
import com.team4099.robot2025.config.constants.ArmConstants.ARM_KV
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.subsystems.arm.ArmIO.ArmIOInputs
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.units.measure.AngularVelocity
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.AngularMechanismSensor
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRotations
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerDegreesPerSecondPerSecond
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.perSecond
import edu.wpi.first.units.measure.AngularAcceleration as WPILibAngularAcceleration
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object ArmIOTalonFX : ArmIO {
  private val armTalon: TalonFX = TalonFX(Constants.Arm.ARM_MOTOR_ID)
  private val armConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val absoluteEncoder: CANcoder = CANcoder(Constants.Arm.CANCODER_ID)
  private val absoluteEncoderConfiguration: MagnetSensorConfigs = MagnetSensorConfigs()

  var armAccelerationStatusSignal: StatusSignal<WPILibAngularAcceleration>
  var armTempStatusSignal: StatusSignal<WPILibTemperature>
  var armAppliedVoltageStatusSignal: StatusSignal<WPILibVoltage>
  var armStatorCurrentStatusSignal: StatusSignal<WPILibCurrent>
  var armSupplyCurrentStatusSignal: StatusSignal<WPILibCurrent>
  var absoluteEncoderSignal: StatusSignal<edu.wpi.first.units.measure.Angle>

  private var armPositionSignal: StatusSignal<edu.wpi.first.units.measure.Angle>
  private var armVelocitySignal: StatusSignal<AngularVelocity>

  private var motionMagicTargetVelocity: StatusSignal<Double>
  private var motionMagicTargetPosition: StatusSignal<Double>

  val voltageControl: VoltageOut = VoltageOut(0.volts.inVolts)
  val positionControl: MotionMagicVoltage = MotionMagicVoltage(0.degrees.inDegrees)

  val armSensor: AngularMechanismSensor =
    ctreAngularMechanismSensor(
      armTalon, ArmConstants.ARM_GEAR_RATIO, ArmConstants.VOLTAGE_COMPENSATION
    )

  init {

    // Configure PID Values
    armConfiguration.Slot0.kP =
      armSensor.proportionalPositionGainToRawUnits(ArmConstants.PID.REAL_ARM_KP)
    armConfiguration.Slot0.kI =
      armSensor.integralPositionGainToRawUnits(ArmConstants.PID.REAL_ARM_KI)
    armConfiguration.Slot0.kD =
      armSensor.derivativePositionGainToRawUnits(ArmConstants.PID.REAL_ARM_KD)
    armConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine

    // Configure Feedforward Values
    armConfiguration.Slot0.kG = ArmConstants.ARM_KG.inVolts
    armConfiguration.Slot0.kS = ArmConstants.ARM_KS.inVolts
    armConfiguration.Slot0.kA = armSensor.accelerationFeedforwardToRawUnits(ARM_KA)
    armConfiguration.Slot0.kV = armSensor.velocityFeedforwardToRawUnits(ARM_KV)

    /*
    // Configure Gear Ratio and Cancoder Fusing
    armConfiguration.Feedback.FeedbackRemoteSensorID = absoluteEncoder.deviceID
    armConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.ARM_ENCODER_TO_MECHANISM_RATIO
    armConfiguration.Feedback.RotorToSensorRatio = ArmConstants.ARM_GEAR_RATIO
    armConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder

    // Configure Sensor Direction
    absoluteEncoderConfiguration.SensorDirection = ArmConstants.ENCODER_DIRECTION_VALUE
    absoluteEncoderConfiguration.MagnetOffset = ArmConstants.ENCODER_OFFSET.inRotations


     */

    // Configure Current Limits
    armConfiguration.CurrentLimits.StatorCurrentLimit = ArmConstants.STATOR_CURRENT_LIMIT.inAmperes
    armConfiguration.CurrentLimits.SupplyCurrentLimit = ArmConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    armConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
    armConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true

    // Configure Motor Inversion and Breakmode
    armConfiguration.MotorOutput.Inverted = ArmConstants.INVERSION_VALUE
    armConfiguration.MotorOutput.NeutralMode = ArmConstants.NEUTRAL_MODE_VALUE

    // Configure Motion Magic
    armConfiguration.MotionMagic.MotionMagicAcceleration = ArmConstants.MOTION_MAGIC_ACCELERATION.inDegreesPerSecondPerSecond
    armConfiguration.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MOTION_MAGIC_CRUISE_VELOCITY.inDegreesPerSecond

    // Configure Softlimits
    armConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
      armSensor.positionToRawUnits(ArmConstants.ARM_MAX_ANGLE)
    armConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
      armSensor.positionToRawUnits(ArmConstants.ARM_MIN_ANGLE)

    armConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
    armConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true

    // Apply Configs+

    armTalon.configurator.apply(armConfiguration)
    //absoluteEncoder.configurator.apply(absoluteEncoderConfiguration)

    motionMagicTargetPosition = armTalon.closedLoopReference
    motionMagicTargetVelocity = armTalon.closedLoopReferenceSlope

    motionMagicTargetPosition.setUpdateFrequency(250.0)
    motionMagicTargetVelocity.setUpdateFrequency(250.0)


    armPositionSignal = armTalon.position
    armVelocitySignal = armTalon.velocity
    armAccelerationStatusSignal = armTalon.acceleration
    armTempStatusSignal = armTalon.deviceTemp
    armAppliedVoltageStatusSignal = armTalon.motorVoltage
    armStatorCurrentStatusSignal = armTalon.statorCurrent
    armSupplyCurrentStatusSignal = armTalon.supplyCurrent

    absoluteEncoderSignal = absoluteEncoder.position
  }

  fun updateStatusSignals() {
    BaseStatusSignal.refreshAll(
      armPositionSignal,
      armVelocitySignal,
      motionMagicTargetPosition,
      motionMagicTargetVelocity,
      armAccelerationStatusSignal,
      armTempStatusSignal,
      armAppliedVoltageStatusSignal,
      armStatorCurrentStatusSignal,
      armSupplyCurrentStatusSignal,
      absoluteEncoderSignal
    )
  }

  override fun updateInputs(inputs: ArmIOInputs) {
    updateStatusSignals()

    CustomLogger.recordOutput("Arm/absoluteEncoderRotations", absoluteEncoderSignal.valueAsDouble)

    inputs.armPosition = armSensor.position
    inputs.armVelocity = armSensor.velocity
    inputs.armAcceleration = armAccelerationStatusSignal.valueAsDouble.degrees.perSecond.perSecond
    inputs.armAppliedVoltage = armAppliedVoltageStatusSignal.valueAsDouble.volts
    inputs.armStatorCurrent = armStatorCurrentStatusSignal.valueAsDouble.amps
    inputs.armSupplyCurrent = armSupplyCurrentStatusSignal.valueAsDouble.amps
    inputs.armTemp = armTempStatusSignal.valueAsDouble.celsius

    Logger.recordOutput("Arm/motionMagicPosition", motionMagicTargetPosition.value * ArmConstants.ARM_GEAR_RATIO * 360)
    Logger.recordOutput("Arm/motionMagicVelocity", motionMagicTargetVelocity.value * ArmConstants.ARM_GEAR_RATIO * 360)
  }

  override fun setArmVoltage(voltage: ElectricalPotential) {
    armTalon.setControl(voltageControl.withOutput(voltage.inVolts))
  }

  override fun setArmPosition(position: Angle) {
    armTalon.setControl(positionControl.withPosition(armSensor.positionToRawUnits(position)))
  }

  override fun zeroEncoder() {
    /*
    var angleToZero =
      (absoluteEncoder.position.valueAsDouble).rotations / ArmConstants.ARM_GEAR_RATIO

    CustomLogger.recordOutput("Arm/angleToZero", angleToZero.inDegrees)
    armTalon.setPosition(angleToZero.inRotations)

     */

    armTalon.setPosition(armSensor.positionToRawUnits(ArmConstants.ZERO_OFFSET))

  }

  override fun setArmBrakeMode(brake: Boolean) {
    if (brake) {
      armConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    } else {
      armConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
    }

    armTalon.configurator.apply(armConfiguration)
  }

  override fun configurePID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    val PIDConfig = armConfiguration.Slot0
    PIDConfig.kP = kP.inVoltsPerDegree
    PIDConfig.kI = kI.inVoltsPerDegreeSeconds
    PIDConfig.kD = kD.inVoltsPerDegreePerSecond

    armTalon.configurator.apply(PIDConfig)
  }

  override fun configureFF(
    kG: ElectricalPotential,
    kS: ElectricalPotential,
    kA: AccelerationFeedforward<Radian, Volt>,
    kV: VelocityFeedforward<Radian, Volt>
  ) {
    val FeedforwardConfig = armConfiguration.Slot0
    FeedforwardConfig.kA = kA.inVoltsPerDegreesPerSecondPerSecond
    FeedforwardConfig.kV = kV.inVoltsPerDegreePerSecond
    FeedforwardConfig.kG = kG.inVolts
    FeedforwardConfig.kS = kS.inVolts

    armTalon.configurator.apply(FeedforwardConfig)
  }
}
