package com.team4099.robot2025.subsystems.climber
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.Slot2Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.units.measure.*
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond

object ClimberIOTalon : ClimberIO {
    private val climberTalon: TalonFX = TalonFX(Constants.Climber.CLIMBER_MOTOR_ID)
    private val climberConfiguration: TalonFXConfiguration = TalonFXConfiguration()
    private val climberMotionMagicConfiguration = climberConfiguration.MotionMagic
    private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage((-1337).radians.inRadians)

    private val climberSensor = ctreAngularMechanismSensor(
        climberTalon,
        1.0,
        ClimberConstants.VOLTAGE_COMPENSATION
    )

    private var climberStatorCurrentSignal: StatusSignal<Current>
    private var climberSupplyCurrentSignal: StatusSignal<Current>
    private var climberTempSignal: StatusSignal<Temperature>
    private var climberDutyCycle: StatusSignal<Double>
    private var climberMotorVoltage: StatusSignal<Voltage>
    private var climberMotorTorque: StatusSignal<Current>
    private var climberMotorPosition: StatusSignal<edu.wpi.first.units.measure.Angle>
    private var climberMotorVelocity: StatusSignal<AngularVelocity>
    private var climberMotorAcceleration: StatusSignal<AngularAcceleration>

    private val slot1PositionErrorSwitchThreshold =
        LoggedTunableValue("Climber/slot1PosErrSwitchThreshold", Pair({it.inRadians}, {it.radians}))

    private val slot1VelocitySwitchThreshold = LoggedTunableValue(
        "Climber/slot1VelSwitchThreshold",
        Pair({it.inRadiansPerSecond}, {it.radians.perSecond})
    )

    private val slot2PositionErrorSwitchThreshold =
        LoggedTunableValue("Climber/slot2PosErrSwitchThreshold", Pair({it.inRadians}, {it.radians}))

    private val slot2VelocitySwitchThreshold = LoggedTunableValue(
        "Climber/slot2VelSwitchThreshold",
        Pair({it.inRadiansPerSecond}, {it.radians.perSecond})
    )

    init {
        slot1PositionErrorSwitchThreshold.initDefault(ClimberConstants.PID.SLOT1_POS_SWITCH_THRESHOLD)
        slot1VelocitySwitchThreshold.initDefault(ClimberConstants.PID.SLOT1_VEL_SWITCH_THRESHOLD)
        slot2PositionErrorSwitchThreshold.initDefault(ClimberConstants.PID.SLOT2_POS_SWITCH_THRESHOLD)
        slot2VelocitySwitchThreshold.initDefault(ClimberConstants.PID.SLOT2_VEL_SWITCH_THRESHOLD)

        climberTalon.configurator.apply(TalonFXConfiguration())
        climberTalon.clearStickyFaults()

        climberConfiguration.Slot0.kP =
            climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_REAL)
        climberConfiguration.Slot0.kI =
            climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_REAL)
        climberConfiguration.Slot0.kD =
            climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_REAL)

        climberConfiguration.Slot1.kP =
            climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_UNLATCH)
        climberConfiguration.Slot1.kI =
            climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_UNLATCH)
        climberConfiguration.Slot1.kD =
            climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_UNLATCH)

        climberConfiguration.Slot2.kP = climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_LATCH)
        climberConfiguration.Slot2.kI = climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_LATCH)
        climberConfiguration.Slot2.kD = climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_LATCH)

        climberConfiguration.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT.inAmperes
        climberConfiguration.CurrentLimits.SupplyCurrentLowerLimit = ClimberConstants.THRESHOLD_CURRENT_LIMIT.inAmperes
        climberConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        climberConfiguration.CurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT.inAmperes
        climberConfiguration.CurrentLimits.StatorCurrentLimitEnable = false
        climberMotionMagicConfiguration.MotionMagicCruiseVelocity = 80.0
        climberMotionMagicConfiguration.MotionMagicAcceleration = 160.0

        climberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        climberConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        climberTalon.configurator.apply(climberConfiguration)

        climberStatorCurrentSignal = climberTalon.statorCurrent
        climberSupplyCurrentSignal = climberTalon.supplyCurrent
        climberDutyCycle = climberTalon.dutyCycle
        climberTempSignal = climberTalon.deviceTemp
        climberMotorVoltage = climberTalon.motorVoltage
        climberMotorTorque = climberTalon.torqueCurrent
        climberMotorPosition = climberTalon.position
        climberMotorVelocity = climberTalon.velocity
        climberMotorAcceleration = climberTalon.acceleration
    }

    override fun zeroEncoder() {
        climberTalon.setPosition(0.0)
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
    }

    override fun setVoltage(voltage: ElectricalPotential) {
        climberTalon.setControl(VoltageOut(voltage.inVolts))
    }
    
    override fun setPosition(position: Angle, feedforward: ElectricalPotential, latched: Boolean) {
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

        climberTalon.setControl(
            motionMagicControl
                .withPosition(climberSensor.positionToRawUnits(position))
                .withSlot(slot)
        )
    }

    private fun updateSignals() {
        BaseStatusSignal.refreshAll(
            climberMotorPosition,
            climberMotorVelocity,
            climberMotorAcceleration,
            climberMotorTorque,
            climberMotorVoltage,
            climberDutyCycle,
            climberStatorCurrentSignal,
            climberSupplyCurrentSignal,
            climberTempSignal,
        )
    }

    override fun updateInputs(inputs: ClimberIO.ClimberInputs) {
        updateSignals()

        climberTalon.rotorPosition.refresh()
        climberTalon.position.refresh()

        inputs.climberPosition = climberSensor.position
        inputs.climberVelocity = climberSensor.velocity
        inputs.climberAcceleration = climberMotorAcceleration.valueAsDouble.radians.perSecond.perSecond
        inputs.climberTorque = climberMotorTorque.valueAsDouble
        inputs.climberAppliedVoltage = climberMotorVoltage.valueAsDouble.volts
        inputs.climberDutyCycle = climberDutyCycle.valueAsDouble.volts
        inputs.climberStatorCurrent = climberStatorCurrentSignal.valueAsDouble.amps
        inputs.climberSupplyCurrent = climberSupplyCurrentSignal.valueAsDouble.amps
        inputs.climberTemperature = climberTempSignal.valueAsDouble.celsius

        if (inputs.climberPosition < ClimberConstants.MIN_ANGLE) {
            zeroEncoder()
        }
    }
}