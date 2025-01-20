package com.team4099.robot2025.subsystems.superstructure

import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.rollers.Rollers
import com.team4099.robot2025.subsystems.arm.Arm
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Superstructure(
  private val drivetrain: Drivetrain,
  private val elevator: Elevator,
  private val rollers: Rollers,
  private val arm: Arm,
  private val limelight: LimelightVision

  ) : SubsystemBase() {
  var currentRequest: Request.SuperstructureRequest = Request.SuperstructureRequest.Idle()
  var currentState: SuperstructureStates = SuperstructureStates.UNINITIALIZED

  var isAtRequestedState: Boolean = false;

  override fun periodic() {
    // Made to put code in bc we will be using this eventually
    var nextState = currentState
    when (currentState) {
      SuperstructureStates.UNINITIALIZED -> {
        nextState = SuperstructureStates.HOME_PREP
      }

      SuperstructureStates.HOME_PREP -> {
        /* zero needed subsystems */
        arm.currentRequest = Request.ArmRequest.Zero()

        if (arm.isZeroed) {
          nextState = SuperstructureStates.HOME
        }

        if (currentRequest is Request.SuperstructureRequest.Tuning) {
          nextState = SuperstructureStates.TUNING
        }
      }

      SuperstructureStates.HOME -> {
        elevator.currentRequest = Request.ElevatorRequest.Home()

        if (elevator.isHomed) {
          nextState = SuperstructureStates.IDLE
        }
      }

      SuperstructureStates.IDLE -> {

      }

      SuperstructureStates.PREP_SCORE_CORAL ->  {

      }
    }
  }



  companion object {
    enum class SuperstructureStates {
      UNINITIALIZED,
      TUNING,
      IDLE,
      HOME_PREP,
      HOME,
      PREP_INTAKE_CORAL,
      INTAKE_CORAL,
      PREP_INTAKE_ALGAE,
      INTAKE_ALGAE,
      PREP_SCORE_ALGAE_PROCESSOR,
      SCORE_ALGAE_PROCESSOR,
      PREP_SCORE_CORAL,
      SCORE_CORAL,
      CLIMB_EXTEND,
      CLIMB_RETRACT
      // PREP_SCORE_BARGE
      // SCORE_BARGE (ommited for now to avoid non-exhaustive when statements).
    }
  }
}
