package com.team4099.robot2025.subsystems.elevator

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.perSecond
import com.team4099.robot2025.subsystems.superstructure.Request.ElevatorRequest as ElevatorRequest

class Elevator(val io: ElevatorIO) {
  val inputs = ElevatorIO.ElevatorInputs()

  var targetVoltage: ElectricalPotential = 0.0.volts

  var homingCurrentSpikeStartTime = 0.0.seconds

  val upperLimitReached: Boolean
    get() = inputs.elevatorPosition >= ElevatorConstants.UPWARDS_EXTENSION_LIMIT

  val lowerLimitReached: Boolean
    get() = inputs.elevatorPosition <= ElevatorConstants.DOWNWARDS_EXTENSION_LIMIT

  var isHomed = false

  var currentState: ElevatorState = ElevatorState.UNINITIALIZED
  var currentRequest: ElevatorRequest = ElevatorRequest.OpenLoop(0.0.volts)
    set(value) {
      when (value) {
        is ElevatorRequest.OpenLoop -> elevatorVoltageTarget = value.voltage
        is ElevatorRequest.ClosedLoop -> {
          elevatorPositionTarget = value.position
        }
        else -> {}
      }
      field = value
    }

  var elevatorPositionTarget = 0.0.inches
    private set
  var elevatorVelocityTarget = 0.0.inches.perSecond
    private set
  var elevatorVoltageTarget = 0.0.volts
    private set

  private var lastRequestedPosition = (-9999).inches
  private var lastRequestedVelocity = -9999.inches.perSecond
  private var lastRequestedVoltage = (-9999).volts

  private var lastHomingStatorCurrentTripTime = Clock.fpgaTime

  private val isAtTargetedPosition: Boolean
    get() =
      (
        currentRequest is ElevatorRequest.ClosedLoop &&
          (inputs.elevatorPosition - elevatorPositionTarget).absoluteValue <=
          ElevatorConstants.ELEVATOR_TOLERANCE
        ) ||
        (ElevatorTunableValues.TunableElevatorHeights.enableElevator.get() != 1.0)

  val canContinueSafely: Boolean
    get() =
      (
        currentRequest is ElevatorRequest.ClosedLoop &&
          (
            (
              (inputs.elevatorPosition - elevatorPositionTarget).absoluteValue <=
                ElevatorConstants.ELEVATOR_SAFE_THRESHOLD
              ) &&
              lastRequestedPosition == elevatorPositionTarget
            )
        )

  init {
    ElevatorTunableValues.TunableElevatorHeights

    if (RobotBase.isReal()) {
      isHomed = false

      ElevatorTunableValues.slot0kP.initDefault(ElevatorConstants.PID.REAL_KP)
      ElevatorTunableValues.slot0kI.initDefault(ElevatorConstants.PID.REAL_KI)
      ElevatorTunableValues.slot0kD.initDefault(ElevatorConstants.PID.REAL_KD)
    } else {
      isHomed = true

      ElevatorTunableValues.slot0kP.initDefault(ElevatorConstants.PID.SIM_KP)
      ElevatorTunableValues.slot0kI.initDefault(ElevatorConstants.PID.SIM_KI)
      ElevatorTunableValues.slot0kD.initDefault(ElevatorConstants.PID.SIM_KD)

      io.configFirstStagePID(ElevatorTunableValues.slot0kP.get(), ElevatorTunableValues.slot0kI.get(),ElevatorTunableValues.slot0kD.get())
      io.configSecondStagePID(ElevatorTunableValues.slot1kP.get(), ElevatorTunableValues.slot1kI.get(), ElevatorTunableValues.slot1kD.get())
    }
  }

  fun periodic() {
    io.updateInputs(inputs)

    if (ElevatorTunableValues.slot0kP.hasChanged() || ElevatorTunableValues.slot0kI.hasChanged() || ElevatorTunableValues.slot0kD.hasChanged()) {
      io.configFirstStagePID(ElevatorTunableValues.slot0kP.get(), ElevatorTunableValues.slot0kI.get(), ElevatorTunableValues.slot0kD.get())
    }
    if (ElevatorTunableValues.slot1kD.hasChanged() || ElevatorTunableValues.slot1kP.hasChanged() || ElevatorTunableValues.slot1kI.hasChanged()) {
      io.configSecondStagePID(ElevatorTunableValues.slot1kP.get(), ElevatorTunableValues.slot1kI.get(), ElevatorTunableValues.slot1kD.get())
    }

    CustomLogger.processInputs("Elevator", inputs)

    CustomLogger.recordOutput(
      "Elevator/elevatorHeight",
      (inputs.elevatorPosition + ElevatorConstants.ELEVATOR_GROUND_OFFSET).inInches
    )

    CustomLogger.recordOutput("Elevator/currentState", currentState.name)
    CustomLogger.recordOutput("Elevator/currentRequest", currentRequest.javaClass.simpleName)

    CustomLogger.recordDebugOutput("Elevator/isHomed", isHomed)
    CustomLogger.recordDebugOutput("Elevator/canContinueSafely", canContinueSafely)

    CustomLogger.recordDebugOutput("Elevator/isAtTargetPosition", isAtTargetedPosition)

    CustomLogger.recordDebugOutput(
      "Elevator/elevatorPositionTarget", elevatorPositionTarget.inInches
    )
    CustomLogger.recordDebugOutput(
      "Elevator/elevatorVelocityTarget", elevatorVelocityTarget.inInchesPerSecond
    )
    CustomLogger.recordDebugOutput("Elevator/elevatorVoltageTarget", elevatorVoltageTarget.inVolts)

    CustomLogger.recordDebugOutput(
      "Elevator/lastElevatorPositionTarget", lastRequestedPosition.inInches
    )
    CustomLogger.recordDebugOutput(
      "Elevator/lastElevatorVelocityTarget", lastRequestedVelocity.inInchesPerSecond
    )
    CustomLogger.recordDebugOutput(
      "Elevator/lastElevatorVoltageTarget", lastRequestedVoltage.inVolts
    )

    CustomLogger.recordDebugOutput("Elevator/upperLimitReached", upperLimitReached)
    CustomLogger.recordDebugOutput("Elevator/lowerLimitReached", lowerLimitReached)

    var nextState = currentState

    when (currentState) {
      ElevatorState.UNINITIALIZED -> {
        nextState = fromElevatorRequestToState(currentRequest)
        io.zeroEncoder()
      }
      ElevatorState.HOME -> {
        if (inputs.leaderStatorCurrent < ElevatorConstants.HOMING_STALL_CURRENT) {
          lastHomingStatorCurrentTripTime = Clock.fpgaTime
        }

        if (!inputs.isSimulating &&
          (
            !isHomed &&
              inputs.leaderStatorCurrent < ElevatorConstants.HOMING_STALL_CURRENT &&
              (Clock.fpgaTime - lastHomingStatorCurrentTripTime) <
              ElevatorConstants.HOMING_STALL_TIME_THRESHOLD
            )
        ) {
          setVoltage(ElevatorConstants.HOMING_APPLIED_VOLTAGE)
        } else {
          io.zeroEncoder()
          isHomed = true
        }

        if (isHomed) {
          nextState = fromElevatorRequestToState(currentRequest)
        }
      }
      ElevatorState.OPEN_LOOP -> {
        setVoltage(targetVoltage)
        nextState = fromElevatorRequestToState(currentRequest)
      }
      ElevatorState.CLOSED_LOOP -> {
        if (elevatorPositionTarget != lastRequestedPosition ||
          elevatorVelocityTarget != lastRequestedVelocity
        ) {

          lastRequestedPosition = elevatorPositionTarget
          lastRequestedVelocity = elevatorVelocityTarget
        }

        io.setPosition(elevatorPositionTarget)

        nextState = fromElevatorRequestToState(currentRequest)

        if (!(currentState.equivalentToRequest(currentRequest))) {
          lastRequestedPosition = (-1337).inches
          lastRequestedVelocity = -1337.inches.perSecond
        }
      }
    }

    currentState = nextState
  }

  fun setVoltage(targetVoltage: ElectricalPotential) {
    if ((upperLimitReached && targetVoltage > 0.0.volts) ||
      (lowerLimitReached && targetVoltage < 0.0.volts)
    ) {
      io.setVoltage(0.0.volts)
    } else {
      io.setVoltage(targetVoltage)
    }
  }

  companion object {
    enum class ElevatorState {
      UNINITIALIZED,
      OPEN_LOOP,
      CLOSED_LOOP,
      HOME;

      inline fun equivalentToRequest(request: ElevatorRequest): Boolean {
        return (request is ElevatorRequest.Home && this == HOME) ||
          (request is ElevatorRequest.OpenLoop && this == OPEN_LOOP) ||
          (request is ElevatorRequest.ClosedLoop && this == CLOSED_LOOP)
      }
    }

    inline fun fromElevatorRequestToState(request: ElevatorRequest): ElevatorState {
      return when (request) {
        is ElevatorRequest.Home -> ElevatorState.HOME
        is ElevatorRequest.OpenLoop -> ElevatorState.OPEN_LOOP
        is ElevatorRequest.ClosedLoop -> ElevatorState.CLOSED_LOOP
      }
    }
  }
}
