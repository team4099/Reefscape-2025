package com.team4099.robot2025.subsystems.arm

// import com.team4099.lib.math.clamp
// import com.team4099.robot2025.config.constants.ArmConstants
// import com.team4099.robot2025.config.constants.Constants
// import com.team4099.robot2025.subsystems.arm.ArmIO.ArmIOInputs
// import edu.wpi.first.math.system.plant.DCMotor
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
// import org.team4099.lib.controller.ProfiledPIDController
// import org.team4099.lib.controller.TrapezoidProfile
// import org.team4099.lib.units.base.Current
// import org.team4099.lib.units.base.amps
// import org.team4099.lib.units.base.celsius
// import org.team4099.lib.units.base.inMeters
// import org.team4099.lib.units.base.inSeconds
// import org.team4099.lib.units.derived.inKilogramsMeterSquared
// import org.team4099.lib.units.derived.inRadians
// import org.team4099.lib.units.derived.inVolts
// import org.team4099.lib.units.derived.radians
// import org.team4099.lib.units.derived.volts
// import org.team4099.lib.units.perSecond

// TODO: completely broken

object ArmIOSim : ArmIO {
  /*
   private val armSim: SingleJointedArmSim =
     SingleJointedArmSim(
       DCMotor.getKrakenX60(1),
       1.0 / ArmConstants.ARM_GEAR_RATIO,
       ArmConstants.ARM_INERTIA.inKilogramsMeterSquared,
       ArmConstants.ARM_LENGTH.inMeters,
       ArmConstants.ARM_MIN_ANGLE.inRadians,
       ArmConstants.ARM_MAX_ANGLE.inRadians,
       true,
       0.0
     )

   private val armPIDController =
     ProfiledPIDController(
       ArmConstants.PID.SIM_ARM_KP,
       ArmConstants.PID.SIM_ARM_KI,
       ArmConstants.PID.SIM_ARM_KD,
       TrapezoidProfile.Constraints(ArmConstants.SIM_VELOCITY, ArmConstants.SIM_ACCELERATION)
     )

   private var appliedVoltage = 0.0.volts

   init {
     armSim.setState(0.0, 0.0)
   }

   override fun updateInputs(inputs: ArmIOInputs) {
     armSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

     inputs.armPosition = armSim.angleRads.radians
     inputs.armVelocity = armSim.velocityRadPerSec.radians.perSecond
     inputs.armAcceleration = 0.0.radians.perSecond.perSecond
     inputs.armAppliedVoltage = appliedVoltage
     inputs.armStatorCurrent = armSim.currentDrawAmps.amps
     inputs.armSupplyCurrent = 0.amps
     inputs.armTemp = 0.0.celsius
     inputs.isSimulating = true
   }

   override fun setArmCurrent(amps: Current) {
     val clampedVoltage =
       clamp(voltage, -ArmConstants.VOLTAGE_COMPENSATION, ArmConstants.VOLTAGE_COMPENSATION)
     armSim.setInputVoltage(clampedVoltage.inVolts)
     appliedVoltage = clampedVoltage
   }

   override fun setArmBrakeMode(brake: Boolean) {}
  */
}
