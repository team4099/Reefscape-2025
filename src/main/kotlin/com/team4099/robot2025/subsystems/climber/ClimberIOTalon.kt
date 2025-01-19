package com.team4099.robot2025.subsystems.climber
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
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
import org.team4099.lib.units.inRotationsPerSecond
import org.team4099.lib.units.inRotationsPerSecondPerSecond
import org.team4099.lib.units.perSecond

object ClimberIOTalon : ClimberIO {
    private val climberTalon: TalonFX = TalonFX(Constants.Climber.CLIMBER_MOTOR_ID)
    private val climberConfiguration: TalonFXConfiguration = TalonFXConfiguration()

    private val climberSensor = ctreAngularMechanismSensor(
        climberTalon,
        1.0,
        ClimberConstants.VOLTAGE_COMPENSATION
    )

    private val motionMagicConfiguration = climberConfiguration.MotionMagic
    private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage((-1337).degrees.inDegrees)

    private var statorCurrentSignal: StatusSignal<Current>
    private var supplyCurrentSignal: StatusSignal<Current>
    private var tempSignal: StatusSignal<Temperature>
    private var dutyCycle: StatusSignal<Double>
    private var motorVoltage: StatusSignal<Voltage>
    private var motorTorque: StatusSignal<Current>
    private var motorPosition: StatusSignal<edu.wpi.first.units.measure.Angle>
    private var motorVelocity: StatusSignal<AngularVelocity>
    private var motorAcceleration: StatusSignal<AngularAcceleration>

    init {
        climberTalon.configurator.apply(TalonFXConfiguration())
        climberTalon.clearStickyFaults()

        climberConfiguration.Slot0.kP =
            climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_UNLATCH)
        climberConfiguration.Slot0.kI =
            climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_UNLATCH)
        climberConfiguration.Slot0.kD =
            climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_UNLATCH)

        climberConfiguration.Slot1.kP =
            climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_UNLATCH)
        climberConfiguration.Slot1.kI =
            climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_UNLATCH)
        climberConfiguration.Slot1.kD =
            climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_UNLATCH)

        climberConfiguration.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT.inAmperes
        climberConfiguration.CurrentLimits.SupplyCurrentLowerLimit = ClimberConstants.THRESHOLD_CURRENT_LIMIT.inAmperes
        climberConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        climberConfiguration.CurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT.inAmperes
        climberConfiguration.CurrentLimits.StatorCurrentLimitEnable = false
        motionMagicConfiguration.MotionMagicCruiseVelocity = ClimberConstants.MAX_VELOCITY.inRotationsPerSecond
        motionMagicConfiguration.MotionMagicAcceleration =
            ClimberConstants.MAX_ACCELERATION.inRotationsPerSecondPerSecond

        climberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        climberConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        climberTalon.configurator.apply(climberConfiguration)

        statorCurrentSignal = climberTalon.statorCurrent
        supplyCurrentSignal = climberTalon.supplyCurrent
        dutyCycle = climberTalon.dutyCycle
        tempSignal = climberTalon.deviceTemp
        motorVoltage = climberTalon.motorVoltage
        motorTorque = climberTalon.torqueCurrent
        motorPosition = climberTalon.position
        motorVelocity = climberTalon.velocity
        motorAcceleration = climberTalon.acceleration
    }

    override fun zeroEncoder() {
        climberTalon.setPosition(0.0)
    }

    override fun configPIDSlot0(
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

    override fun setVoltage(voltage: ElectricalPotential) {
        climberTalon.setControl(VoltageOut(voltage.inVolts))
    }
    
    override fun setPosition(position: Angle, feedforward: ElectricalPotential, latched: Boolean) {
        val slot = if (latched) 0 else 1

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
            motorPosition,
            motorVelocity,
            motorAcceleration,
            motorTorque,
            motorVoltage,
            dutyCycle,
            statorCurrentSignal,
            supplyCurrentSignal,
            tempSignal,
        )
    }

    override fun updateInputs(inputs: ClimberIO.ClimberInputs) {
        updateSignals()

        climberTalon.rotorPosition.refresh()
        climberTalon.position.refresh()

        inputs.climberPosition = climberSensor.position
        inputs.climberVelocity = climberSensor.velocity
        inputs.climberAcceleration = motorAcceleration.valueAsDouble.degrees.perSecond.perSecond
        inputs.climberTorque = motorTorque.valueAsDouble.newtons
        inputs.climberAppliedVoltage = motorVoltage.valueAsDouble.volts
        inputs.climberDutyCycle = dutyCycle.valueAsDouble.volts
        inputs.climberStatorCurrent = statorCurrentSignal.valueAsDouble.amps
        inputs.climberSupplyCurrent = supplyCurrentSignal.valueAsDouble.amps
        inputs.climberTemperature = tempSignal.valueAsDouble.celsius

        if (inputs.climberPosition < ClimberConstants.MIN_ANGLE) {
            zeroEncoder()
        }
    }
}