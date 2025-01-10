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
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.arm.ArmIO.ArmIOInputs
import com.team4099.robot2025.util.CustomLogger
import org.team4099.lib.units.AngularMechanismSensor
import edu.wpi.first.units.measure.AngularAcceleration as WPILibAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPILibAngularVelocity
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perSecond
import edu.wpi.first.units.measure.Angle as WPILibAngle

object ArmIOTalonFX: ArmIO {
    private val armTalon: TalonFX = TalonFX(Constants.Arm.ARM_MOTOR_ID)
    private val armConfiguration: TalonFXConfiguration = TalonFXConfiguration()

    private val absoluteEncoder: CANcoder = CANcoder(Constants.Arm.CANCODER_ID)
    private val absoluteEncoderConfiguration: MagnetSensorConfigs = MagnetSensorConfigs()

    var armPositionStatusSignal: StatusSignal<WPILibAngle>
    var armVelocityStatusSignal: StatusSignal<WPILibAngularVelocity>
    var armAccelerationStatusSignal: StatusSignal<WPILibAngularAcceleration>
    var armTempStatusSignal: StatusSignal<WPILibTemperature>
    var armAppliedVoltageStatusSignal: StatusSignal<WPILibVoltage>
    var armStatorCurrentStatusSignal: StatusSignal<WPILibCurrent>
    var armSupplyCurrentStatusSignal: StatusSignal<WPILibCurrent>

    val voltageControl: VoltageOut = VoltageOut(-1337.volts.inVolts)
    val positionControl: MotionMagicVoltage = MotionMagicVoltage(-1337.degrees.inDegrees)

    val armMechanismSensor: AngularMechanismSensor = ctreAngularMechanismSensor(armTalon, ArmConstants.ARM_GEAR_RATIO, 0.volts)

    var isSimulating = false

    init {

        // Configure PID Values
        armConfiguration.Slot0.kP = armMechanismSensor.proportionalPositionGainToRawUnits(ArmConstants.ARM_KP)
        armConfiguration.Slot0.kI = armMechanismSensor.integralPositionGainToRawUnits(ArmConstants.ARM_KI)
        armConfiguration.Slot0.kD = armMechanismSensor.derivativePositionGainToRawUnits(ArmConstants.ARM_KD)
        armConfiguration.Slot0.GravityType = ArmConstants.GRAVITY_TYPE
        // Configure Gear Ratio and Cancoder Fusing
        armConfiguration.Feedback.FeedbackRemoteSensorID = Constants.Arm.CANCODER_ID
        armConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.ARM_ENCODER_RATIO
        armConfiguration.Feedback.RotorToSensorRatio = ArmConstants.ARM_GEAR_RATIO
        armConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder

        // Configure Current Limits
        armConfiguration.CurrentLimits.StatorCurrentLimit = ArmConstants.STATOR_CURRENT_LIMIT.inAmperes
        armConfiguration.CurrentLimits.SupplyCurrentLimit = ArmConstants.SUPPLY_CURRENT_LIMIT.inAmperes
        armConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
        armConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true

        // Configure Motor Inversion and Breakmode
        armConfiguration.MotorOutput.Inverted = ArmConstants.INVERSION_VALUE
        armConfiguration.MotorOutput.NeutralMode = ArmConstants.NEUTRAL_MODE_VALUE

        // Configure Motion Magic
        armConfiguration.MotionMagic.MotionMagicAcceleration = armMechanismSensor.accelerationToRawUnits(ArmConstants.MOTION_MAGIC_ACCELERATION)
        armConfiguration.MotionMagic.MotionMagicCruiseVelocity = armMechanismSensor.velocityToRawUnits(ArmConstants.MOTION_MAGIC_CRUISE_VELOCITY)

        // Configure Softlimits
        armConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = armMechanismSensor.positionToRawUnits(ArmConstants.FORWRARD_SOFT_LIMIT_THRESHOLD)
        armConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = armMechanismSensor.positionToRawUnits(ArmConstants.REVERSE_SOFT_LIMIT_THRESHOLD)

        armConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
        armConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true

        // Configure Sensor Direction
        absoluteEncoderConfiguration.SensorDirection = ArmConstants.ENCODER_DIRECTION_VALUE
        absoluteEncoderConfiguration.MagnetOffset = armMechanismSensor.positionToRawUnits(ArmConstants.ENCODER_OFFSET)

        armTalon.configurator.apply(armConfiguration)
        absoluteEncoder.configurator.apply(absoluteEncoderConfiguration)


        // Initilize Motor Input Data
        armPositionStatusSignal = armTalon.position
        armVelocityStatusSignal = armTalon.velocity
        armAccelerationStatusSignal = armTalon.acceleration
        armTempStatusSignal = armTalon.deviceTemp
        armAppliedVoltageStatusSignal = armTalon.motorVoltage
        armStatorCurrentStatusSignal = armTalon.statorCurrent
        armSupplyCurrentStatusSignal = armTalon.supplyCurrent
    }

    fun updateStatusSignals() {
        BaseStatusSignal.refreshAll(
            armPositionStatusSignal,
            armVelocityStatusSignal,
            armAccelerationStatusSignal,
            armTempStatusSignal,
            armAppliedVoltageStatusSignal,
            armStatorCurrentStatusSignal,
            armSupplyCurrentStatusSignal)
    }

    override fun updateInputs(inputs: ArmIOInputs) {
        updateStatusSignals()

        inputs.armPosition = armPositionStatusSignal.valueAsDouble.degrees
        inputs.armVelocity = armVelocityStatusSignal.valueAsDouble.degrees.perSecond
        inputs.armAcceleration = armAccelerationStatusSignal.valueAsDouble.degrees.perSecond.perSecond
        inputs.armAppliedVoltage = armAppliedVoltageStatusSignal.valueAsDouble.volts
        inputs.armStatorCurrent = armStatorCurrentStatusSignal.valueAsDouble.amps
        inputs.armSupplyCurrent = armSupplyCurrentStatusSignal.valueAsDouble.amps
        inputs.armTemp = armTempStatusSignal.valueAsDouble.celsius
    }

    override fun setArmVoltage(voltage: ElectricalPotential) {
        armTalon.setControl(
            voltageControl.withOutput(voltage.inVolts))
    }

    override fun setArmPosition(position: Angle) {
        armTalon.setControl(
            positionControl.withPosition(armMechanismSensor.positionToRawUnits(position)).withSlot(0))
    }

    override fun zeroEncoder() {
        var angleToZero =
            (absoluteEncoder.position.valueAsDouble).rotations /
            ArmConstants.ARM_GEAR_RATIO

        CustomLogger.recordOutput("Arm/angleToZero", angleToZero.inDegrees)
        armTalon.setPosition(angleToZero.inRotations)
    }

    override fun setArmBrakeMode(brake: Boolean) {
        if (brake) {
            armConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        }
        else {
            armConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
        }

        armTalon.configurator.apply(armConfiguration)
    }
}