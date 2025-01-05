package com.team4099.robot2023.subsystems.intake

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.util.CustomLogger
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Intake(val io: IntakeIO) : SubsystemBase() {
  val inputs = IntakeIO.IntakeIOInputs()

  object TunableIntakeStates {
    val idleRollerVoltage =
      LoggedTunableValue(
        "Intake/idleRollerVoltage",
        IntakeConstants.IDLE_ROLLER_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
    val idleCenterWheelVoltage =
      LoggedTunableValue(
        "Intake/idleCenterWheelVoltage",
        IntakeConstants.IDLE_CENTER_WHEEL_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val intakeRollerVoltage =
      LoggedTunableValue(
        "Intake/intakeRollerVoltage",
        IntakeConstants.INTAKE_ROLLER_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val intakeCenterWheelVoltage =
      LoggedTunableValue(
        "Intake/intakeCenterWheelVoltage",
        IntakeConstants.INTAKE_CENTER_WHEEL_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val outtakeRolllerVoltage =
      LoggedTunableValue(
        "Intake/outtakeRollerVoltage",
        IntakeConstants.OUTTAKE_ROLLER_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val outtakeCenterWheelVoltage =
      LoggedTunableValue(
        "Intake/outtakeCenterWheelVoltage",
        IntakeConstants.OUTTAKE_CENTER_WHEEL_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val testRollerVoltage =
      LoggedTunableValue(
        "Intake/testVoltage",
        IntakeConstants.INTAKE_ROLLER_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val testCenterWheelVoltage =
      LoggedTunableValue(
        "Intake/testVoltage",
        IntakeConstants.INTAKE_CENTER_WHEEL_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
  }

  var rollerVoltageTarget: ElectricalPotential = 0.0.volts
  var centerWheelVoltageTarget: ElectricalPotential = 0.0.volts

  var isZeroed = false
  var currentState: IntakeState = IntakeState.UNINITIALIZED

  var currentRequest: Request.IntakeRequest =
    Request.IntakeRequest.OpenLoop(
      IntakeConstants.IDLE_ROLLER_VOLTAGE, IntakeConstants.IDLE_CENTER_WHEEL_VOLTAGE
    )
    set(value) {
        when (value) {
          is Request.IntakeRequest.OpenLoop -> {
            rollerVoltageTarget = value.rollerVoltage
            centerWheelVoltageTarget = value.centerWheelVoltage
          }
        }

        field = value
      }

  private var timeProfileGeneratedAt = Clock.fpgaTime

  override fun periodic() {
    io.updateInputs(inputs)

    Logger.processInputs("Intake", inputs)
    Logger.recordOutput("Intake/currentState", currentState.name)
    Logger.recordOutput("Intake/requestedState", currentRequest.javaClass.simpleName)
    Logger.recordOutput("Intake/isZeroed", isZeroed)

    if (Constants.Tuning.DEBUGING_MODE) {
      Logger.recordOutput(
        "Intake/isAtCommandedState", currentState.equivalentToRequest(currentRequest)
      )
      CustomLogger.recordDebugOutput(
        "Intake/timeProfileGeneratedAt", timeProfileGeneratedAt.inSeconds
      )
      Logger.recordOutput("Intake/rollerVoltageTarget", rollerVoltageTarget.inVolts)
      Logger.recordOutput("Intake/centerWheelVoltageTarget", centerWheelVoltageTarget.inVolts)
    }

    var nextState = currentState
    when (currentState) {
      IntakeState.UNINITIALIZED -> {
        // Outputs
        // No designated output functionality because targeting position will take care of it next
        // loop cycle

        // Transitions
        nextState = IntakeState.OPEN_LOOP
      }
      IntakeState.OPEN_LOOP -> {
        setVoltage(rollerVoltageTarget, centerWheelVoltageTarget)

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
    }

    // The next loop cycle, we want to run ground intake at the state that was requested. setting
    // current state to the next state ensures that we run the logic for the state we want in the
    // next loop cycle.
    currentState = nextState
  }

  /** @param appliedVoltage Represents the applied voltage of the roller motor */
  fun setVoltage(rollerVoltage: ElectricalPotential, centerWheelVoltage: ElectricalPotential) {
    io.setVoltage(rollerVoltage, centerWheelVoltage)
  }

  fun generateIntakeTestCommand(): Command {
    val returnCommand = runOnce {
      currentRequest =
        Request.IntakeRequest.OpenLoop(
          TunableIntakeStates.testRollerVoltage.get(),
          TunableIntakeStates.testCenterWheelVoltage.get()
        )
    }
    return returnCommand
  }

  companion object {
    enum class IntakeState {
      UNINITIALIZED,
      OPEN_LOOP;

      fun equivalentToRequest(request: Request.IntakeRequest): Boolean {
        return ((request is Request.IntakeRequest.OpenLoop && this == OPEN_LOOP))
      }
    }

    fun fromRequestToState(request: Request.IntakeRequest): IntakeState {
      return when (request) {
        is Request.IntakeRequest.OpenLoop -> IntakeState.OPEN_LOOP
      }
    }
  }
}
