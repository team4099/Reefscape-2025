package com.team4099.robot2025.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.IntakeConstants


import edu.wpi.first.units.measure.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerRadian
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianSeconds
import org.team4099.lib.units.derived.inVoltsPerRadiansPerSecondPerSecond
import org.team4099.lib.units.derived.perRadian
import org.team4099.lib.units.derived.perRadianPerSecond
import org.team4099.lib.units.derived.perRadianSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import edu.wpi.first.units.measure.AngularAcceleration as WPILibAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPILibAngularVelocity
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object IntakeIOTalonFX : IntakeIO { //extends our IntakeIO class, since that is the blueprint that creates an instance of our motor
    private val intakeTalon: TalonFX = TalonFX(Constants.Intake.INTAKE_MOTOR_ID) //sets up motor's unique ID
    private val intakeConfiguration: TalonFXConfiguration = TalonFXConfiguration() //sets up the configuration, which we will later config in the INIT

    private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.degrees.inDegrees)  //sets up motionMagic

    //signal the motor sends with these values, will be updated in the periodic
    var intakeAccelerationStatusSignal: StatusSignal<WPILibAngularAcceleration>
    var intakeAppliedVoltageStatusSignal: StatusSignal<WPILibVoltage>
    var intakeStatorCurrentStatusSignal: StatusSignal<WPILibCurrent>
    var intakeSupplyCurrentStatusSignal: StatusSignal<WPILibCurrent>
    var intakeTempStatusSignal: StatusSignal<WPILibTemperature>

    var slot0Configs = intakeConfiguration.Slot0

    private val intakeSensor =
        ctreAngularMechanismSensor(
            intakeTalon, IntakeConstants.GEAR_RATIO, IntakeConstants.VOLTAGE_COMPENSATION
        )

    //these are the settings of the intakeSensor, or atl what sets it up, what motor, what gearRatio, and max amt of voltage it can use.

    var intakeVelocitySignal: StatusSignal<WPILibAngularVelocity>

    init {
        intakeTalon.clearStickyFaults()

        intakeConfiguration.Slot0.kP =
            intakeSensor.proportionalPositionGainToRawUnits(IntakeConstants.PID.REAL_INTAKE_KP)
        intakeConfiguration.Slot0.kI =
            intakeSensor.integralPositionGainToRawUnits(IntakeConstants.PID.REAL_INTAKE_KI)
        intakeConfiguration.Slot0.kD =
            intakeSensor.derivativePositionGainToRawUnits(IntakeConstants.PID.REAL_INTAKE_KD)
        intakeConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine

        // Configure Feedforward Values
        intakeConfiguration.Slot0.kG = IntakeConstants.PID.INTAKE_KG.inVolts
        intakeConfiguration.Slot0.kS = IntakeConstants.PID.INTAKE_KS.inVolts
        intakeConfiguration.Slot0.kA =
            intakeSensor.accelerationFeedforwardToRawUnits(IntakeConstants.PID.INTAKE_KA)
        intakeConfiguration.Slot0.kV =
            intakeSensor.velocityFeedforwardToRawUnits(IntakeConstants.PID.INTAKE_KV)
        //Config Motor's Current limits
        intakeConfiguration.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT.inAmperes
        intakeConfiguration.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT.inAmperes
        intakeConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
        intakeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true

        //Config Motor's inversion feature, and break/coast mode
        //Why is the start at neutral? and how can we change it?
        intakeConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        intakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

        intakeTalon.configurator.apply(intakeConfiguration) //getter function takes in this value from StatusSignal and puts and logs it back into teh table. ts is how it achieves that. And this happens 20ms in periodic

        intakeVelocitySignal = intakeTalon.velocity
        intakeTempStatusSignal = intakeTalon.deviceTemp
        intakeSupplyCurrentStatusSignal = intakeTalon.supplyCurrent
        intakeStatorCurrentStatusSignal = intakeTalon.statorCurrent
        intakeAccelerationStatusSignal = intakeTalon.acceleration
        intakeAppliedVoltageStatusSignal = intakeTalon.motorVoltage
    }

    fun updateStatusSignals() {
        BaseStatusSignal.refreshAll(
            intakeVelocitySignal,
            intakeAccelerationStatusSignal,
            intakeTempStatusSignal,
            intakeAppliedVoltageStatusSignal,
            intakeStatorCurrentStatusSignal,
            intakeSupplyCurrentStatusSignal,
        )
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) { //converts to doubles, so it is usable now
        updateStatusSignals() //uses our updated status signals, to update our logTable.

        inputs.intakePosition = intakeSensor.position
        inputs.intakeAcceleration = intakeAccelerationStatusSignal.valueAsDouble.degrees.perSecond.perSecond
        inputs.intakeAppliedVoltage = intakeAppliedVoltageStatusSignal.valueAsDouble.volts
        inputs.intakeStatorCurrent = intakeStatorCurrentStatusSignal.valueAsDouble.amps
        inputs.intakeSupplyCurrent = intakeSupplyCurrentStatusSignal.valueAsDouble.amps
        inputs.intakeTemp = intakeTempStatusSignal.valueAsDouble.celsius
    }

    override fun setIntakeVoltage(volts: ElectricalPotential) { //This is what communicates with hardware, and is needed for logic's open loop
        intakeTalon.setControl(VoltageOut(volts.inVolts))
    }

    override fun setIntakePosition(position: Angle) {
        val slotused = 0
        intakeTalon.setControl(
            motionMagicControl.withPosition(intakeSensor.positionToRawUnits(position)).withSlot(slotused)
        )
    }
    override fun setIntakeBrakeMode(brake: Boolean) {
        if (brake) {
            intakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        } else {
            intakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
        }
        intakeTalon.configurator.apply(intakeConfiguration) //resets the motor's configs, use it whenever you make changes to the motor config.
    }
    //changes mode of the motor between brake, and coast however we typically just set it at brake mode. (im assuming prior to match in config)

    override fun configPID(
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {
        val configPID = slot0Configs
        configPID.kP = kP.inVoltsPerRadian
        configPID.kI = kI.inVoltsPerRadianSeconds
        configPID.kD = kD.inVoltsPerRadianPerSecond
    }


    override fun configFF(
        kG: ElectricalPotential,
        kS: StaticFeedforward<Volt>,
        kV: VelocityFeedforward<Radian, Volt>,
        kA: AccelerationFeedforward<Radian, Volt>
    ) {
        val FFConfig = slot0Configs
        FFConfig.kG = kG.inVolts
        FFConfig.kS = kS.inVolts
        FFConfig.kA = kA.inVoltsPerRadiansPerSecondPerSecond
        FFConfig.kV = kV.inVoltsPerRadianPerSecond

        intakeTalon.configurator.apply(FFConfig)
    }
}
