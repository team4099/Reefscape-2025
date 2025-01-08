package com.team4099.robot2025.subsystems.climber
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.phoenix6.PositionVoltage
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond

object ClimberIOTalon : ClimberIO {
    private val climberTalon: TalonFX = TalonFX(Constants.Climber.CLIMBER_MOTOR_ID)
    private val framePerimeterTalon: TalonFX = TalonFX(Constants.Climber.FRAME_PERIMETER_MOTOR_ID)
    private val climberConfiguration: TalonFXConfiguration = TalonFXConfiguration()
    private val framePerimeterConfiguration: TalonFXConfiguration = TalonFXConfiguration()

    private val absoluteEncoder: CANcoder = CANcoder(Constants.Climber.CANCODER_ID)
    private val absoluteEncoderConfiguration: MagnetSensorConfigs = MagnetSensorConfigs()

    var positionRequest = PositionVoltage((-1337).radians, slot = 0, feedforward = (-1337).volts)

    //TODO: Add pulleys when design is fully completed

    val slot1PositionErrorSwitchThreshold =
        LoggedTunableValue("Climber/slot1PosErrSwitchThreshold", Pair({it.inRadians}, {it.radians}))
    val slot1VelocitySwitchThreshold =
        LoggedTunableValue("Climber/slot1VelSwitchThreshold", Pair({it.inRadiansPerSecond}, {it.radians.perSecond}))

    val slot2PositionErrorSwitchThreshold =
        LoggedTunableValue("Climber/slot2PosErrSwitchThreshold", Pair({it.inRadians}, {it.radians}))
    val slot2VelocitySwitchThreshold =
        LoggedTunableValue("Climber/slot2VelSwitchThreshold", Pair({it.inRadiansPerSecond}, {it.radians.perSecond}))

    private val climberSensor = ctreAngularMechanismSensor(
        climberTalon,
        1.0,
        ClimberConstants.VOLTAGE_COMPENSATION
    )

    private val framePerimeterSensor = ctreAngularMechanismSensor(
        framePerimeterTalon,
        1.0,
        ClimberConstants.VOLTAGE_COMPENSATION
    )

    private var climberStatorCurrentSignal: StatusSignal<Current>
    private var climberSupplyCurrentSignal: StatusSignal<Current>
    private var climberTempSignal: StatusSignal<Temperature>
    private var climberDutyCycle: StatusSignal<Double>
    private var climberMotorVoltage: StatusSignal<Voltage>
    private var climberMotorTorque: StatusSignal<Current>
    private var climberMotorAcceleration: StatusSignal<AngularAcceleration>
    private var climberAbsoluteEncoderSignal: StatusSignal<edu.wpi.first.units.measure.Angle>
    private var framePerimeterStatorCurrentSignal: StatusSignal<Current>
    private var framePerimeterSupplyCurrentSignal: StatusSignal<Current>
    private var framePerimeterTempSignal: StatusSignal<Temperature>
    private var framePerimeterDutyCycle: StatusSignal<Double>
    private var framePerimeterMotorVoltage: StatusSignal<Voltage>
    private var framePerimeterMotorTorque: StatusSignal<Current>
    private var framePerimeterMotorAcceleration: StatusSignal<AngularAcceleration>
    private var framePerimeterAbsoluteEncoderSignal: StatusSignal<edu.wpi.first.units.measure.Angle>

    var angleToZero: Angle = 0.0.radians

    init {
        slot1PositionErrorSwitchThreshold.initDefault(ClimberConstants.PID.SLOT1_POS_SWITCH_THRESHOLD)
        slot1VelocitySwitchThreshold.initDefault(ClimberConstants.PID.SLOT1_VEL_SWITCH_THRESHOLD)
        slot2PositionErrorSwitchThreshold.initDefault(ClimberConstants.PID.SLOT2_POS_SWITCH_THRESHOLD)
        slot2VelocitySwitchThreshold.initDefault(ClimberConstants.PID.SLOT2_VEL_SWITCH_THRESHOLD)

        climberTalon.configurator.apply(TalonFXConfiguration())
        framePerimeterTalon.configurator.apply(TalonFXConfiguration())
        climberTalon.clearStickyFaults()
        framePerimeterTalon.clearStickyFaults()

        // TODO: Check if AbsoluteSensorDiscontinuityPoint is able to replace AbsoluteSensorRange
        // AbsoluteSensorDiscontinuityPoint replaces AbsoluteSensorRange in Phoenix5
        absoluteEncoderConfiguration.AbsoluteSensorDiscontinuityPoint = 0.5
        absoluteEncoderConfiguration.SensorDirection = SensorDirectionValue.Clockwise_Positive
        absoluteEncoderConfiguration.MagnetOffset = ClimberConstants.ABSOLUTE_ENCODER_OFFSET.inRotations
        absoluteEncoder.configurator.apply(absoluteEncoderConfiguration)

        climberConfiguration.Feedback.FeedbackRemoteSensorID = absoluteEncoder.deviceID
        climberConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
        climberConfiguration.Feedback.SensorToMechanismRatio = ClimberConstants.ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO
        climberConfiguration.Feedback.RotorToSensorRatio = ClimberConstants.MOTOR_TO_ABSOLUTE_ENCODER_GEAR_RATIO
        framePerimeterConfiguration.Feedback.FeedbackRemoteSensorID = absoluteEncoder.deviceID
        framePerimeterConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
        framePerimeterConfiguration.Feedback.SensorToMechanismRatio = ClimberConstants.ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO
        framePerimeterConfiguration.Feedback.RotorToSensorRatio = ClimberConstants.MOTOR_TO_ABSOLUTE_ENCODER_GEAR_RATIO

        climberConfiguration.Slot0.kP = climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_REAL)
        climberConfiguration.Slot0.kI = climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_REAL)
        climberConfiguration.Slot0.kD = climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_REAL)
        framePerimeterConfiguration.Slot0.kP = climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_REAL)
        framePerimeterConfiguration.Slot0.kI = climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_REAL)
        framePerimeterConfiguration.Slot0.kD = climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_REAL)

        climberConfiguration.Slot1.kP = climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_UNLATCH)
        climberConfiguration.Slot1.kI = climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_UNLATCH)
        climberConfiguration.Slot1.kD = climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_UNLATCH)
        framePerimeterConfiguration.Slot1.kP = climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_UNLATCH)
        framePerimeterConfiguration.Slot1.kI = climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_UNLATCH)
        framePerimeterConfiguration.Slot1.kD = climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_UNLATCH)

        climberConfiguration.Slot2.kP = climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_LATCH)
        climberConfiguration.Slot2.kI = climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_LATCH)
        climberConfiguration.Slot2.kD = climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_LATCH)
        framePerimeterConfiguration.Slot2.kP = climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_LATCH)
        framePerimeterConfiguration.Slot2.kI = climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_LATCH)
        framePerimeterConfiguration.Slot2.kD = climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_LATCH)

        // TODO: Check if these are tunable values or not
        climberConfiguration.Voltage.PeakForwardVoltage = 3.0
        climberConfiguration.Voltage.PeakReverseVoltage = -3.0
        framePerimeterConfiguration.Voltage.PeakForwardVoltage = 3.0
        framePerimeterConfiguration.Voltage.PeakReverseVoltage = -3.0

        // TODO: Find replacements for SupplyCurrentThreshold
        climberConfiguration.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT.inAmperes
        // climberConfiguration.CurrentLimits.SupplyCurrentThreshold = ClimberConstants.THRESHOLD_CURRENT_LIMIT.inAmperes
        climberConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        climberConfiguration.CurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT.inAmperes
        climberConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

        framePerimeterConfiguration.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT.inAmperes
        // framePerimeterConfiguration.CurrentLimits.SupplyCurrentThreshold = ClimberConstants.THRESHOLD_CURRENT_LIMIT.inAmperes
        framePerimeterConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        framePerimeterConfiguration.CurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT.inAmperes
        framePerimeterConfiguration.CurrentLimits.StatorCurrentLimitEnable = false


        climberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        climberConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        framePerimeterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        framePerimeterConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

        climberTalon.configurator.apply(climberConfiguration)
        framePerimeterTalon.configurator.apply(framePerimeterConfiguration)

        climberStatorCurrentSignal = climberTalon.statorCurrent
        climberSupplyCurrentSignal = climberTalon.supplyCurrent
        climberDutyCycle = climberTalon.dutyCycle
        climberTempSignal = climberTalon.deviceTemp
        climberMotorVoltage = climberTalon.motorVoltage
        climberMotorTorque = climberTalon.torqueCurrent
        climberMotorAcceleration = climberTalon.acceleration
        climberAbsoluteEncoderSignal = absoluteEncoder.position

        framePerimeterStatorCurrentSignal = framePerimeterTalon.statorCurrent
        framePerimeterSupplyCurrentSignal = framePerimeterTalon.supplyCurrent
        framePerimeterDutyCycle = framePerimeterTalon.dutyCycle
        framePerimeterTempSignal = framePerimeterTalon.deviceTemp
        framePerimeterMotorVoltage = framePerimeterTalon.motorVoltage
        framePerimeterMotorTorque = framePerimeterTalon.torqueCurrent
        framePerimeterMotorAcceleration = framePerimeterTalon.acceleration
        framePerimeterAbsoluteEncoderSignal = absoluteEncoder.position

        // TODO: Add MotorChecker when falconspin is fixed
    }

    override fun configPID(
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {
        val climberPIDConfig = Slot0Configs()
        climberPIDConfig.kP = climberSensor.proportionalPositionGainToRawUnits(kP)
        climberPIDConfig.kI = climberSensor.integralPositionGainToRawUnits(kI)
        climberPIDConfig.kD = climberSensor.derivativePositionGainToRawUnits(kD)
        climberTalon.configurator.apply(climberPIDConfig)

        val framePerimeterPIDConfig = Slot0Configs()
        framePerimeterPIDConfig.kP = climberSensor.proportionalPositionGainToRawUnits(kP)
        framePerimeterPIDConfig.kI = climberSensor.integralPositionGainToRawUnits(kI)
        framePerimeterPIDConfig.kD = climberSensor.derivativePositionGainToRawUnits(kD)
        framePerimeterTalon.configurator.apply(framePerimeterPIDConfig)
    }

    override fun configPIDSlot1 (
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {
        val climberPIDConfig = Slot1Configs()
        climberPIDConfig.kP = climberSensor.proportionalPositionGainToRawUnits(kP)
        climberPIDConfig.kI = climberSensor.integralPositionGainToRawUnits(kI)
        climberPIDConfig.kD = climberSensor.derivativePositionGainToRawUnits(kD)
        climberTalon.configurator.apply(climberPIDConfig)

        val framePerimeterPIDConfig = Slot1Configs()
        framePerimeterPIDConfig.kP = climberSensor.proportionalPositionGainToRawUnits(kP)
        framePerimeterPIDConfig.kI = climberSensor.integralPositionGainToRawUnits(kI)
        framePerimeterPIDConfig.kD = climberSensor.derivativePositionGainToRawUnits(kD)
        framePerimeterTalon.configurator.apply(framePerimeterPIDConfig)
    }

    override fun configPIDSlot2 (
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {
        val climberPIDConfig = Slot2Configs()
        climberPIDConfig.kP = climberSensor.proportionalPositionGainToRawUnits(kP)
        climberPIDConfig.kI = climberSensor.integralPositionGainToRawUnits(kI)
        climberPIDConfig.kD = climberSensor.derivativePositionGainToRawUnits(kD)
        climberTalon.configurator.apply(climberPIDConfig)

        val framePerimeterPIDConfig = Slot2Configs()
        framePerimeterPIDConfig.kP = climberSensor.proportionalPositionGainToRawUnits(kP)
        framePerimeterPIDConfig.kI = climberSensor.integralPositionGainToRawUnits(kI)
        framePerimeterPIDConfig.kD = climberSensor.derivativePositionGainToRawUnits(kD)
        framePerimeterTalon.configurator.apply(framePerimeterPIDConfig)
    }

    override fun setClimberVoltage(voltage: ElectricalPotential) {
        climberTalon.setControl(VoltageOut(voltage.inVolts))
    }

    override fun setFramePerimeterVoltage(voltage: ElectricalPotential) {
        framePerimeterTalon.setControl(VoltageOut(voltage.inVolts))
    }
    
    override fun setClimberPosition(position: Angle, feedforward: ElectricalPotential, latched: Boolean) {
        positionRequest.setFeedforward(feedforward)
        positionRequest.setPosition(position)

        val currentError = climberSensor.position - position
        val currentVelocity = climberSensor.velocity

        var slot = if (latched) 0 else 1


        if (currentError.absoluteValue <= slot2PositionErrorSwitchThreshold.get() &&
            currentVelocity.absoluteValue <= slot2VelocitySwitchThreshold.get())
        {
            slot = 2
        }

        CustomLogger.recordOutput("Climber/feedForwardApplied", feedforward.inVolts)
        CustomLogger.recordOutput("Climber/slotBeingUsed", slot)

        // TODO: Fix PositionDutyCycle parameters
        /*
        climberTalon.setControl(
            PositionDutyCycle(
                climberSensor.positionToRawUnits(position),
                climberSensor.velocityToRawUnits(0.radians.perSecond),
                true,
                feedforward.inVolts,
                slot,
                false,
                false,
                false
            )
        )
         */
    }

    override fun setFramePerimeterPosition(position: Angle, feedforward: ElectricalPotential, latched: Boolean) {

    }

    private fun updateSignals() {
        BaseStatusSignal.refreshAll(
            climberMotorTorque,
            climberMotorVoltage,
            climberDutyCycle,
            climberSupplyCurrentSignal,
            climberStatorCurrentSignal,
            climberTempSignal,
            climberMotorAcceleration,
            climberAbsoluteEncoderSignal
        )

        BaseStatusSignal.refreshAll(
            framePerimeterMotorTorque,
            framePerimeterMotorVoltage,
            framePerimeterDutyCycle,
            framePerimeterSupplyCurrentSignal,
            framePerimeterStatorCurrentSignal,
            framePerimeterTempSignal,
            framePerimeterMotorAcceleration,
            framePerimeterAbsoluteEncoderSignal
        )
    }

    override fun updateInputs(inputs: ClimberIO.ClimberInputs) {
        updateSignals()

        climberTalon.rotorPosition.refresh()
        framePerimeterTalon.rotorPosition.refresh()
        climberTalon.position.refresh()
        framePerimeterTalon.position.refresh()

        /*
        CustomLogger.recordOutput("Climber/climberRotorTMechanismRadians",
            (
                    (
                            climberTalon.rotorPosition.value /
                                    ClimberConstants.MOTOR_TO_ABSOLUTE_ENCODER_GEAR_RATIO /
                                    (1.06488 / 1.0)
                            ) + angleToZero
                    )
                .inRadians)
         */
    }
}