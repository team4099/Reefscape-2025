package com.team4099.robot2025.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.subsystems.falconspin.MotorChecker
import com.team4099.robot2025.subsystems.falconspin.MotorCollection
import edu.wpi.first.wpilibj.motorcontrol.Talon
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.derived.*


object ElevatorIOTalon : ElevatorIO {
    private val leaderTalon: TalonFX = TalonFX(Constants.Elevator.LEADER_MOTOR_ID)
    private val followerTalon: TalonFX = TalonFX(Constants.Elevator.FOLLOWER_MOTOR_ID)

    private val leaderConfigs: TalonFXConfiguration = TalonFXConfiguration()
    private val followerConfigs: TalonFXConfiguration = TalonFXConfiguration()

    private val leaderSensor = ctreLinearMechanismSensor(
        leaderTalon,
        ElevatorConstants.ELEVATOR_PULLEY_TO_MOTOR,
        ElevatorConstants.SPOOL_DIAMETER,
        ElevatorConstants.VOLTAGE_COMPENSATION
    )

    private val followerSensor = ctreLinearMechanismSensor(
        followerTalon,
        ElevatorConstants.ELEVATOR_PULLEY_TO_MOTOR,
        ElevatorConstants.SPOOL_DIAMETER,
        ElevatorConstants.VOLTAGE_COMPENSATION
    )


    var leaderStatorCurrentSignal: StatusSignal<Double>
    var leaderSupplyCurrentSignal: StatusSignal<Double>
    var leaderTempSignal: StatusSignal<Double>
    var leaderDutyCycle: StatusSignal<Double>

    var followerStatorCurrentSignal: StatusSignal<Double>
    var followerSupplyCurrentSignal: StatusSignal<Double>
    var followerTempSignal: StatusSignal<Double>
    var followerDutyCycle: StatusSignal<Double>
    var motorVoltage: StatusSignal<Double>
    var motorTorque: StatusSignal<Double>


    init {

        leaderTalon.clearStickyFaults()
        followerTalon.clearStickyFaults()

        leaderTalon.configurator.apply(TalonFXConfiguration())
        followerTalon.configurator.apply(TalonFXConfiguration())

        leaderTalon.clearStickyFaults()
        followerTalon.clearStickyFaults()

        leaderConfigs.Slot0.kP = leaderSensor.proportionalPositionGainToRawUnits(ElevatorConstants.PID.REAL_KP)
        leaderConfigs.Slot0.kI = leaderSensor.integralPositionGainToRawUnits(ElevatorConstants.PID.REAL_KI)
        leaderConfigs.Slot0.kD = leaderSensor.derivativePositionGainToRawUnits(ElevatorConstants.PID.REAL_KD)

        followerConfigs.Slot0.kP = followerSensor.proportionalPositionGainToRawUnits(ElevatorConstants.PID.REAL_KP)
        followerConfigs.Slot0.kI = followerSensor.integralPositionGainToRawUnits(ElevatorConstants.PID.REAL_KI)
        followerConfigs.Slot0.kD = followerSensor.derivativePositionGainToRawUnits(ElevatorConstants.PID.REAL_KD)




        leaderConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT
        leaderConfigs.CurrentLimits.SupplyCurrentLowerLimit = ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT
        leaderConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.LEADER_STATOR_CURRENT_LIMIT
        leaderConfigs.CurrentLimits.SupplyCurrentLowerTime = ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT
        leaderConfigs.CurrentLimits.StatorCurrentLimitEnable = true
        leaderConfigs.CurrentLimits.SupplyCurrentLimitEnable = true


        followerConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.FOLLOWER_SUPPLY_CURRENT_LIMIT
        followerConfigs.CurrentLimits.SupplyCurrentLowerLimit = ElevatorConstants.FOLLOWER_SUPPLY_CURRENT_LIMIT
        followerConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.FOLLOWER_STATOR_CURRENT_LIMIT
        followerConfigs.CurrentLimits.SupplyCurrentLowerTime = ElevatorConstants.FOLLOWER_SUPPLY_CURRENT_LIMIT
        followerConfigs.CurrentLimits.StatorCurrentLimitEnable = true
        followerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true



        //TODO: figure this shit out
        leaderStatorCurrentSignal = leaderTalon.statorCurrent.valueAsDouble as StatusSignal<Double>
        leaderSupplyCurrentSignal = leaderTalon.supplyCurrent.valueAsDouble
        leaderTempSignal = leaderTalon.deviceTemp.valueAsDouble
        leaderDutyCycle = leaderTalon.dutyCycle


        followerStatorCurrentSignal = followerTalon.statorCurrent.valueAsDouble
        followerSupplyCurrentSignal = followerTalon.supplyCurrent.valueAsDouble
        followerTempSignal = followerTalon.deviceTemp.valueAsDouble
        followerDutyCycle = followerTalon.dutyCycle

        motorVoltage = leaderTalon.motorVoltage.valueAsDouble
        motorTorque = leaderTalon.torqueCurrent.valueAsDouble

        MotorChecker.add(
            "Elevator",
            "Extension",
            MotorCollection(
                mutableListOf(
                    Talon(leaderTalon, "Leader Extension Motor"), Talon(followerTalon, "Follower Extension Motor")
                ),
                ElevatorConstants.LEADER_STATOR_CURRENT_LIMIT,
                90.celsius,
                ElevatorConstants.LEADER_STATOR_CURRENT_LIMIT - 30.amps,
                110.celsius
            ),
        )
    }

    private fun updateSignals() {
        BaseStatusSignal.refreshAll(
            motorTorque,
            motorVoltage,
            leaderTempSignal,
            leaderDutyCycle,
            leaderStatorCurrentSignal,
            leaderSupplyCurrentSignal,


            followerTempSignal,
            followerDutyCycle,
            followerStatorCurrentSignal,
            followerSupplyCurrentSignal,

            )

    }

    override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {

        updateSignals()

        inputs.elevatorPosition = leaderSensor.position
        inputs.elevatorVelocity = leaderSensor.velocity

        inputs.leaderTemperature = leaderTalon.deviceTemp.valueAsDouble.celsius
        inputs.leaderSupplyCurrent = leaderTalon.supplyCurrent.valueAsDouble.amps
        inputs.leaderStatorCurrent = leaderTalon.statorCurrent.valueAsDouble.amps
        inputs.leaderAppliedVoltage = leaderTalon.dutyCycle.valueAsDouble.volts


        inputs.followerTemperature = followerTalon.deviceTemp.valueAsDouble.celsius
        inputs.followerStatorCurrent = followerTalon.statorCurrent.valueAsDouble.amps
        inputs.followerSupplyCurrent = followerTalon.supplyCurrent.valueAsDouble.amps
        inputs.followerAppliedVoltage = followerTalon.dutyCycle.valueAsDouble.volts
    }

    override fun configPID(
        kP: ProportionalGain<Meter, Volt>, kI: IntegralGain<Meter, Volt>, kD: DerivativeGain<Meter, Volt>
    ) {
        val pidConfiguration = Slot0Configs()
        pidConfiguration.kP = leaderSensor.proportionalPositionGainToRawUnits(kP)
        pidConfiguration.kI = leaderSensor.integralPositionGainToRawUnits(kI)
        pidConfiguration.kD = leaderSensor.derivativePositionGainToRawUnits(kD)

        leaderTalon.configurator.apply(pidConfiguration)
    }

    override fun setPosition(position: Length, feedforward: ElectricalPotential) {

    }

    override fun setVoltage(targetVoltage: ElectricalPotential) {

    }

    override fun zeroEncoder() {
        leaderTalon.setPosition(0.0)
        followerTalon.setPosition(0.0)
    }
}
