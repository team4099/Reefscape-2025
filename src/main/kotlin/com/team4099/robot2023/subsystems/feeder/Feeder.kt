package com.team4099.robot2023.subsystems.feeder

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FeederConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.util.CustomLogger
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Feeder(val io: FeederIO) : SubsystemBase() {
  val inputs = FeederIO.FeederIOInputs()

  var rumbleTrigger = false

  var rumbleTime = 0.5.seconds

  val rumbleStartTime = Clock.fpgaTime

  var lastHeldGamePiece = false

  var lastDropTime = Clock.fpgaTime

  object TunableFeederStates {
    val idleVoltage =
      LoggedTunableValue(
        "Feeder/idleVoltage", FeederConstants.IDLE_VOLTAGE, Pair({ it.inVolts }, { it.volts })
      )
    val intakeVoltage =
      LoggedTunableValue(
        "Feeder/intakeVoltage",
        FeederConstants.INTAKE_NOTE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
    val outtakeVoltage =
      LoggedTunableValue(
        "Feeder/outtakeVoltage",
        FeederConstants.OUTTAKE_NOTE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
    val autoIntakeVoltage =
      LoggedTunableValue(
        "Feeder/autoIntakeVoltage",
        FeederConstants.AUTO_INTAKE_NOTE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
    val shootVoltage =
      LoggedTunableValue(
        "Feeder/shootVoltage",
        FeederConstants.SHOOT_NOTE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val testIntakeVoltage =
      LoggedTunableValue(
        "Feeder/testIntakeVoltage",
        FeederConstants.INTAKE_NOTE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val testShootVoltage =
      LoggedTunableValue(
        "Feeder/testShootVoltage",
        FeederConstants.SHOOT_NOTE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
  }

  var feederTargetVoltage: ElectricalPotential = 0.0.volts
  var lastFeederRunTime = 0.0.seconds

  private val shootNoteVoltage =
    LoggedTunableValue(
      "Feeder/ShootNoteVoltage",
      FeederConstants.SHOOT_NOTE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  var currentState: FeederStates = FeederStates.UNINITIALIZED
  var currentRequest: Request.FeederRequest =
    Request.FeederRequest.OpenLoopIntake(FeederConstants.FEEDER_IDLE_VOLTAGE)
    set(value) {
        when (value) {
          is Request.FeederRequest.OpenLoopIntake -> {
            feederTargetVoltage = value.feederVoltage
          }
          is Request.FeederRequest.OpenLoopShoot -> {
            feederTargetVoltage = value.feederVoltage
          }
          else -> {}
        }
        field = value
      }

  val noteDetected: Boolean
    get() {
      return inputs.feederVelocity.absoluteValue <= FeederConstants.NOTE_VELOCITY_THRESHOLD &&
        inputs.feederStatorCurrent > 10.amps &&
        inputs.feederAppliedVoltage.sign < 0 &&
        (Clock.fpgaTime - lastFeederRunTime) >=
        FeederConstants.WAIT_BEFORE_DETECT_VELOCITY_DROP || inputs.isSimulated
    }

  var firstTripBeamBreakTime = Clock.fpgaTime

  var lastBeamState = false

  var debounceFilter = Debouncer(FeederConstants.beamBreakFilterTime.inSeconds)

  var hasNote: Boolean = false

  private var timeProfileGeneratedAt = Clock.fpgaTime

  override fun periodic() {
    io.updateInputs(inputs)

    if (lastHeldGamePiece != hasNote && !rumbleTrigger) {
      rumbleTrigger = true
      lastDropTime = Clock.fpgaTime
    }

    if (Clock.fpgaTime - lastDropTime > rumbleTime) {
      rumbleTrigger = false
    }

    lastHeldGamePiece = hasNote

    hasNote = debounceFilter.calculate(inputs.beamBroken)

    Logger.recordOutput("Feeder/hasNote", hasNote)

    Logger.processInputs("Feeder", inputs)
    Logger.recordOutput("Feeder/currentState", currentState)
    Logger.recordOutput("Feeder/requestedState", currentRequest.javaClass.simpleName)

    if (Constants.Tuning.DEBUGING_MODE) {
      Logger.recordOutput(
        "Feeder/isAtCommandedState", currentState.equivalentToRequest(currentRequest)
      )
      CustomLogger.recordDebugOutput(
        "Feeder/timeProfileGeneratedAt", timeProfileGeneratedAt.inSeconds
      )
      Logger.recordOutput("Feeder/feederVoltageTarget", feederTargetVoltage.inVolts)
    }

    if (!lastBeamState && inputs.beamBroken) {
      firstTripBeamBreakTime = Clock.fpgaTime
    }

    lastBeamState = inputs.beamBroken

    var nextState = currentState
    when (currentState) {
      FeederStates.UNINITIALIZED -> {
        nextState = FeederStates.OPEN_LOOP_INTAKE
      }
      FeederStates.OPEN_LOOP_INTAKE -> {
        setFeederVoltage(feederTargetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
      FeederStates.OPEN_LOOP_SHOOT -> {
        setFeederVoltage(feederTargetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
    }
    currentState = nextState

    Logger.recordOutput("Feeder/currentState", currentState.toString())
    Logger.recordOutput("Feeder/currentRequest", currentRequest.javaClass.toString())
    Logger.recordOutput("Feeder/targetVoltage", feederTargetVoltage.inVolts)
  }

  fun setFeederVoltage(appliedVoltage: ElectricalPotential) {
    io.setFeederVoltage(appliedVoltage)
  }

  fun feederOpenLoopIntakeTestCommand(): Command {
    return runOnce({
      currentRequest = Request.FeederRequest.OpenLoopIntake(FeederConstants.INTAKE_NOTE_VOLTAGE)
    })
  }

  fun feederOpenLoopShootTestCommand(): Command {
    return runOnce({ currentRequest = Request.FeederRequest.OpenLoopShoot(shootNoteVoltage.get()) })
  }

  companion object {
    enum class FeederStates {
      UNINITIALIZED,
      OPEN_LOOP_INTAKE,
      OPEN_LOOP_SHOOT;

      fun equivalentToRequest(request: Request.FeederRequest): Boolean {
        return (
          (request is Request.FeederRequest.OpenLoopShoot && this == OPEN_LOOP_SHOOT) ||
            (request is Request.FeederRequest.OpenLoopIntake && this == OPEN_LOOP_INTAKE)
          )
      }
    }

    fun fromRequestToState(request: Request.FeederRequest): FeederStates {
      return when (request) {
        is Request.FeederRequest.OpenLoopIntake -> FeederStates.OPEN_LOOP_INTAKE
        is Request.FeederRequest.OpenLoopShoot -> FeederStates.OPEN_LOOP_SHOOT
      }
    }
  }
}
