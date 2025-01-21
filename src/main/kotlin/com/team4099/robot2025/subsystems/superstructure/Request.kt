package com.team4099.robot2025.subsystems.superstructure

import com.team4099.robot2025.config.constants.ArmConstants
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees

sealed interface Request {

  sealed interface SuperstructureRequest : Request

  sealed interface DrivetrainRequest : Request {
    class OpenLoop(
      val angularVelocity: AngularVelocity,
      val driveVector: Pair<LinearVelocity, LinearVelocity>,
      val fieldOriented: Boolean = true
    ) : DrivetrainRequest

    class ClosedLoop(
      var chassisSpeeds: ChassisSpeeds,
      val chassisAccels: ChassisSpeeds =
        edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)
    ) : DrivetrainRequest

    class ZeroSensors(val isInAutonomous: Boolean = false) : DrivetrainRequest
    class Idle : DrivetrainRequest

    class LockWheels : DrivetrainRequest
    class Characterize(val voltage: ElectricalPotential) : DrivetrainRequest
  }

  sealed interface ClimberRequest : Request {
    class OpenLoop(val voltage: ElectricalPotential) : ClimberRequest

    class ClosedLoop(val position: Angle) : ClimberRequest

    class Home() : ClimberRequest

  sealed interface ArmRequest : Request {
    class OpenLoop(val armVoltage: ElectricalPotential) : ArmRequest
    class CloseLoop(
      val armPosition: Angle
    ) : ArmRequest
    class Zero() : ArmRequest
  }

  sealed interface RollersRequest : Request {
    class OpenLoop(val RollersVoltage: ElectricalPotential) : RollersRequest
    class Zero(): RollersRequest

  }
}


