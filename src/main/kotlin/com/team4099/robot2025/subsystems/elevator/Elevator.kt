package com.team4099.robot2025.subsystems.elevator

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.perSecond
import com.team4099.robot2025.subsystems.superstructure.Request.ElevatorRequest as ElevatorRequest

class Elevator(val io: ElevatorIO) : SubsystemBase() {
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
        (ElevatorTunableValues.enableElevator.get() != 1.0)

  init {
    ElevatorTunableValues.TunableElevatorHeights

    if (RobotBase.isReal()) {
      isHomed = false

      ElevatorTunableValues.kP.initDefault(ElevatorConstants.PID.REAL_KP)
      ElevatorTunableValues.kI.initDefault(ElevatorConstants.PID.REAL_KI)
      ElevatorTunableValues.kD.initDefault(ElevatorConstants.PID.REAL_KD)
      ElevatorTunableValues.kSFirst.initDefault(ElevatorConstants.PID.REAL_KS_FIRST_STAGE)
      ElevatorTunableValues.kSSecond.initDefault(ElevatorConstants.PID.REAL_KS_SECOND_STAGE)
      ElevatorTunableValues.kSThird.initDefault(ElevatorConstants.PID.REAL_KS_THIRD_STAGE)
    } else {
      isHomed = true

      ElevatorTunableValues.kP.initDefault(ElevatorConstants.PID.SIM_KP)
      ElevatorTunableValues.kI.initDefault(ElevatorConstants.PID.SIM_KI)
      ElevatorTunableValues.kD.initDefault(ElevatorConstants.PID.SIM_KD)
      ElevatorTunableValues.kSFirst.initDefault(ElevatorConstants.PID.SIM_KS_FIRST_STAGE)
      ElevatorTunableValues.kSSecond.initDefault(ElevatorConstants.PID.SIM_KS_SECOND_STAGE)
      ElevatorTunableValues.kSThird.initDefault(ElevatorConstants.PID.SIM_KS_THIRD_STAGE)
    }

    ElevatorTunableValues.kGFirst.initDefault(ElevatorConstants.PID.KG_FIRST_STAGE)
    ElevatorTunableValues.kVFirst.initDefault(ElevatorConstants.PID.KV_FIRST_STAGE)
    ElevatorTunableValues.kAFirst.initDefault(ElevatorConstants.PID.KA_FIRST_STAGE)
    ElevatorTunableValues.kGSecond.initDefault(ElevatorConstants.PID.KG_SECOND_STAGE)
    ElevatorTunableValues.kVSecond.initDefault(ElevatorConstants.PID.KV_SECOND_STAGE)
    ElevatorTunableValues.kASecond.initDefault(ElevatorConstants.PID.KA_SECOND_STAGE)
    ElevatorTunableValues.kGThird.initDefault(ElevatorConstants.PID.KG_THIRD_STAGE)
    ElevatorTunableValues.kVThird.initDefault(ElevatorConstants.PID.KV_THIRD_STAGE)
    ElevatorTunableValues.kAThird.initDefault(ElevatorConstants.PID.KA_THIRD_STAGE)

    io.configPID(
      ElevatorTunableValues.kP.get(),
      ElevatorTunableValues.kI.get(),
      ElevatorTunableValues.kD.get()
    )
    io.configFFFirstStage(
      ElevatorTunableValues.kGFirst.get(),
      ElevatorTunableValues.kSFirst.get(),
      ElevatorTunableValues.kVFirst.get(),
      ElevatorTunableValues.kAFirst.get()
    )
    io.configFFSecondStage(
      ElevatorTunableValues.kGSecond.get(),
      ElevatorTunableValues.kSSecond.get(),
      ElevatorTunableValues.kVSecond.get(),
      ElevatorTunableValues.kASecond.get()
    )
    io.configFFThirdStage(
      ElevatorTunableValues.kGThird.get(),
      ElevatorTunableValues.kSThird.get(),
      ElevatorTunableValues.kVThird.get(),
      ElevatorTunableValues.kAThird.get()
    )
  }

  override fun periodic() {
    io.updateInputs(inputs)

    if (ElevatorTunableValues.kP.hasChanged() ||
      ElevatorTunableValues.kI.hasChanged() ||
      ElevatorTunableValues.kD.hasChanged()
    ) {
      io.configPID(
        ElevatorTunableValues.kP.get(),
        ElevatorTunableValues.kI.get(),
        ElevatorTunableValues.kD.get()
      )
    }

    if (ElevatorTunableValues.kGFirst.hasChanged() ||
      ElevatorTunableValues.kSFirst.hasChanged() ||
      ElevatorTunableValues.kVFirst.hasChanged() ||
      ElevatorTunableValues.kAFirst.hasChanged()
    ) {
      io.configFFFirstStage(
        ElevatorTunableValues.kGFirst.get(),
        ElevatorTunableValues.kSFirst.get(),
        ElevatorTunableValues.kVFirst.get(),
        ElevatorTunableValues.kAFirst.get()
      )
    }

    if (ElevatorTunableValues.kGSecond.hasChanged() ||
      ElevatorTunableValues.kSSecond.hasChanged() ||
      ElevatorTunableValues.kVSecond.hasChanged() ||
      ElevatorTunableValues.kASecond.hasChanged()
    ) {
      io.configFFSecondStage(
        ElevatorTunableValues.kGSecond.get(),
        ElevatorTunableValues.kSSecond.get(),
        ElevatorTunableValues.kVSecond.get(),
        ElevatorTunableValues.kASecond.get()
      )
    }

    if (ElevatorTunableValues.kGThird.hasChanged() ||
      ElevatorTunableValues.kSThird.hasChanged() ||
      ElevatorTunableValues.kVThird.hasChanged() ||
      ElevatorTunableValues.kAThird.hasChanged()
    ) {
      io.configFFThirdStage(
        ElevatorTunableValues.kGThird.get(),
        ElevatorTunableValues.kSThird.get(),
        ElevatorTunableValues.kVThird.get(),
        ElevatorTunableValues.kAThird.get()
      )
    }

    CustomLogger.processInputs("Elevator", inputs)

    CustomLogger.recordOutput(
      "Elevator/elevatorHeight",
      (inputs.elevatorPosition + ElevatorConstants.ELEVATOR_GROUND_OFFSET).inInches
    )

    CustomLogger.recordOutput("Elevator/currentState", currentState.name)
    CustomLogger.recordOutput("Elevator/currentRequest", currentRequest.javaClass.simpleName)

    CustomLogger.recordDebugOutput("Elevator/isHomed", isHomed)

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
