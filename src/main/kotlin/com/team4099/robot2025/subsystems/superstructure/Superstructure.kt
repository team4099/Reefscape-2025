package com.team4099.robot2025.subsystems.superstructure

import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Superstructure(private val drivetrain: Drivetrain) : SubsystemBase() {
  var currentRequest: Request.SuperstructureRequest = Request.SuperstructureRequest.Idle()
  var currentState: SuperstructureStates = SuperstructureStates.UNINITIALIZED

  override fun periodic() {
    // Made to put code in bc we will be using this eventually
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
