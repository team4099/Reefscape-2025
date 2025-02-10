package com.team4099.robot2025.subsystems.superstructure

import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {

  sealed interface SuperstructureRequest : Request {

    class Idle() : SuperstructureRequest
    class Home() : SuperstructureRequest

    class IntakeCoral() : SuperstructureRequest

    class IntakeL1() : SuperstructureRequest

    class IntakeAlgae(val level: Constants.Universal.AlgaeLevel) : SuperstructureRequest

    class ScorePrepCoral(val level: Constants.Universal.CoralLevel) : SuperstructureRequest

    class ScorePrepAlgaeProcessor() : SuperstructureRequest
    class ScorePrepAlgaeBarge() : SuperstructureRequest

    class Score() : SuperstructureRequest

    class ClimbExtend() : SuperstructureRequest
    class ClimbRetract() : SuperstructureRequest

    class Tuning() : SuperstructureRequest
  }

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

  sealed interface ArmRequest : Request {
    class OpenLoop(val armVoltage: ElectricalPotential) : ArmRequest
    class ClosedLoop(val armPosition: Angle) : ArmRequest
    class Zero() : ArmRequest
  }

  sealed interface ClimberRequest : Request {
    class OpenLoop(val voltage: ElectricalPotential) : ClimberRequest

    class ClosedLoop(val position: Angle) : ClimberRequest

    class Home() : ClimberRequest
  }

  sealed interface RollersRequest : Request {
    class OpenLoop(val voltage: ElectricalPotential) : RollersRequest
  }

  sealed interface RampRequest : Request {
    class OpenLoop(val voltage: ElectricalPotential) : RampRequest
  }

  sealed interface ElevatorRequest : Request {
    class ClosedLoop(val position: Length) : ElevatorRequest
    class OpenLoop(val voltage: ElectricalPotential) : ElevatorRequest
    class Home() : ElevatorRequest
  }
}
