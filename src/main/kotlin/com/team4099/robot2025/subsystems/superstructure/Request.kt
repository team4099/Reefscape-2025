package com.team4099.robot2023.subsystems.superstructure

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

    class GroundIntake() : SuperstructureRequest

    class EjectGamePiece() : SuperstructureRequest

    class PrepScoreAmp() : SuperstructureRequest

    class ScoreAmp() : SuperstructureRequest

    class PrepScoreSpeakerLow() : SuperstructureRequest

    class PrepScoreSpeakerMid() : SuperstructureRequest

    class PrepScoreSpeakerHigh() : SuperstructureRequest

    class ScoreSpeaker() : SuperstructureRequest
    class ScoreSpeakerMid() : SuperstructureRequest
    class ScoreSpeakerHigh() : SuperstructureRequest

    class ManualScoreSpeakerPrep(
      val wristAngle: Angle,
      val flywheelVelocity: AngularVelocity,
      val wristTolerance: Angle
    ) : SuperstructureRequest

    class AutoAim() : SuperstructureRequest

    class PrepTrap() : SuperstructureRequest

    class ScoreTrap() : SuperstructureRequest

    class ClimbExtend() : SuperstructureRequest

    class ClimbRetract() : SuperstructureRequest

    class PassingShot() : SuperstructureRequest

    class UnderStageShot() : SuperstructureRequest

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

  sealed interface IntakeRequest : Request {
    class OpenLoop(
      val rollerVoltage: ElectricalPotential,
      val centerWheelVoltage: ElectricalPotential
    ) : IntakeRequest
  }

  sealed interface FeederRequest : Request {
    class OpenLoopIntake(val feederVoltage: ElectricalPotential) : FeederRequest
    class OpenLoopShoot(val feederVoltage: ElectricalPotential) : FeederRequest
  }

  sealed interface ElevatorRequest : Request {
    class TargetingPosition(val position: Length) : ElevatorRequest
    class OpenLoop(val voltage: ElectricalPotential) : ElevatorRequest
    class Home() : ElevatorRequest
  }
  sealed interface WristRequest : Request {
    class OpenLoop(val wristVoltage: ElectricalPotential) : WristRequest
    class TargetingPosition(
      val wristPosition: Angle,
      val wristTolerance: Angle = WristConstants.WRIST_TOLERANCE
    ) : WristRequest
    class Zero() : WristRequest
  }
  sealed interface FlywheelRequest : Request {
    class OpenLoop(val flywheelVoltage: ElectricalPotential) : FlywheelRequest
    class TargetingVelocity(val flywheelVelocity: AngularVelocity) : FlywheelRequest
  }
}
