package com.team4099.robot2025.subsystems.intake


import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.IntakeConstants
import com.team4099.robot2025.subsystems.climber.ClimberIOSim.setVoltage
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
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
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object IntakeIOSim: IntakeIO { //You're creating an object for your intake, and simulating that object.
    private val intakeSim: SingleJointedArmSim =
        SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            1 / IntakeConstants.GEAR_RATIO,
            IntakeConstants.INERTIA.inKilogramsMeterSquared,
            IntakeConstants.LENGTH.inMeters,
            IntakeConstants.MIN_ROTATION.inRadians,
            IntakeConstants.MAX_ROTATION.inRadians,
            true,
            0.0
        )

    private var appliedVoltage = 0.0.volts  //why are we bringing this back up?

    var intakeTargetPos = -1337.degrees

    private val intakeController =
        PIDController(
            IntakeConstants.PID.SIM_PIVOT_KP,
            IntakeConstants.PID.SIM_PIVOT_KI,
            IntakeConstants.PID.SIM_PIVOT_KD,
        )
    private var intakeFeedforward =
        ArmFeedforward(
            IntakeConstants.PID.INTAKE_KS, IntakeConstants.PID.INTAKE_KG, IntakeConstants.PID.INTAKE_KV, IntakeConstants.PID.INTAKE_KA
        )

    init { //setting position of our intake
        intakeSim.setState(0.0,0.0)
    }
//updating inputs in SIM
    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        intakeSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
        inputs.intakeVelocity = intakeSim.velocityRadPerSec.radians.perSecond
        inputs.intakeAcceleration = 0.0.radians.perSecond.perSecond
        inputs.intakeAppliedVoltage = appliedVoltage
        inputs.intakeStatorCurrent = intakeSim.currentDrawAmps.amps
        inputs.intakeSupplyCurrent = 0.0.amps
        inputs.intakeTemp = 0.0.celsius

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(intakeSim.currentDrawAmps)

        //simulates roboRIO and battery during match
    )
    }

    override fun setIntakeBrakeMode(brake: Boolean) {}
    //sets a voltage min and max
    override fun setIntakeVoltage(volts: ElectricalPotential) {
        val clampedVoltage =
            clamp(volts, -ArmConstants.VOLTAGE_COMPENSATION, ArmConstants.VOLTAGE_COMPENSATION)
        intakeSim.setInputVoltage(clampedVoltage.inVolts)
        appliedVoltage = clampedVoltage
    }

    override fun setIntakePosition(position: Angle) {
        intakeTargetPos = position
        val feedback = intakeController.calculate(intakeSim.angleRads.radians, position)
        val feedforward =
            intakeFeedforward.calculate(
                intakeSim.angleRads.radians, intakeSim.velocityRadPerSec.radians.perSecond
            )
        setVoltage(feedback + feedforward)
    }
    override fun configPID(
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {
        intakeController.setPID(kP, kI, kD)
    }

    override fun configFF(
        kG: ElectricalPotential,
        kS: StaticFeedforward<Volt>,
        kV: VelocityFeedforward<Radian, Volt>,
        kA: AccelerationFeedforward<Radian, Volt>
    ) {
        intakeFeedforward = ArmFeedforward(kS, kG, kV, kA)
    }
}

//Questions: 1. How should I go about debugging code? 3. How do we track position of intake, another variable?