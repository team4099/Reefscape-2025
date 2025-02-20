package com.team4099.robot2025.subsystems.elevator

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.perSecond
import com.team4099.robot2025.subsystems.superstructure.Request.ElevatorRequest as ElevatorRequest

class Elevator(val io: ElevatorIO) : SubsystemBase() {
  val inputs = ElevatorIO.ElevatorInputs()

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

  private var lastHomingStatorCurrentTripTime = Clock.fpgaTime

  val isAtTargetedPosition: Boolean
    get() =
      (
        currentRequest is ElevatorRequest.ClosedLoop &&
          (inputs.elevatorPosition - elevatorPositionTarget).absoluteValue <=
          ElevatorConstants.ELEVATOR_TOLERANCE
        ) ||
        (ElevatorTunableValues.enableElevator.get() != 1.0)

  init {

    if (RobotBase.isReal()) {
      isHomed = false

      ElevatorTunableValues.kP.initDefault(ElevatorConstants.PID.REAL_KP)
      ElevatorTunableValues.kI.initDefault(ElevatorConstants.PID.REAL_KI)
      ElevatorTunableValues.kD.initDefault(ElevatorConstants.PID.REAL_KD)
    } else {
      isHomed = true

      ElevatorTunableValues.kP.initDefault(ElevatorConstants.PID.SIM_KP)
      ElevatorTunableValues.kI.initDefault(ElevatorConstants.PID.SIM_KI)
      ElevatorTunableValues.kD.initDefault(ElevatorConstants.PID.SIM_KD)
    }

    ElevatorTunableValues.kS.initDefault(ElevatorConstants.PID.KS)
    ElevatorTunableValues.kV.initDefault(ElevatorConstants.PID.KV)
    ElevatorTunableValues.kA.initDefault(ElevatorConstants.PID.KA)
    ElevatorTunableValues.kGDefault.initDefault(ElevatorConstants.PID.KG_DEFAULT)
    ElevatorTunableValues.kGFirst.initDefault(ElevatorConstants.PID.KG_FIRST_STAGE)
    ElevatorTunableValues.kGSecond.initDefault(ElevatorConstants.PID.KG_SECOND_STAGE)
    ElevatorTunableValues.kGThird.initDefault(ElevatorConstants.PID.KG_THIRD_STAGE)

    io.configPID(
      ElevatorTunableValues.kP.get(),
      ElevatorTunableValues.kI.get(),
      ElevatorTunableValues.kD.get()
    )

    io.configFF(
      ElevatorTunableValues.kGFirst.get(),
      ElevatorTunableValues.kGSecond.get(),
      ElevatorTunableValues.kGThird.get(),
      ElevatorTunableValues.kS.get(),
      ElevatorTunableValues.kV.get(),
      ElevatorTunableValues.kA.get()
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

    if (ElevatorTunableValues.kGDefault.hasChanged() ||
      ElevatorTunableValues.kS.hasChanged() ||
      ElevatorTunableValues.kV.hasChanged() ||
      ElevatorTunableValues.kA.hasChanged()
    ) {
      io.configFF(
        ElevatorTunableValues.kGFirst.get(),
        ElevatorTunableValues.kGSecond.get(),
        ElevatorTunableValues.kGThird.get(),
        ElevatorTunableValues.kS.get(),
        ElevatorTunableValues.kV.get(),
        ElevatorTunableValues.kA.get()
      )
    }

    CustomLogger.processInputs("Elevator", inputs)

    CustomLogger.recordOutput(
      "Elevator/elevatorHeightWithGroundOffset",
      (inputs.elevatorPosition + ElevatorConstants.ELEVATOR_GROUND_OFFSET).inInches
    )

    CustomLogger.recordOutput("Elevator/currentState", currentState.name)
    CustomLogger.recordOutput("Elevator/currentRequest", currentRequest.javaClass.simpleName)

    CustomLogger.recordOutput("Elevator/isHomed", isHomed)

    CustomLogger.recordOutput("Elevator/isAtTargetPosition", isAtTargetedPosition)

    CustomLogger.recordOutput("Elevator/elevatorPositionTarget", elevatorPositionTarget.inInches)
    CustomLogger.recordOutput(
      "Elevator/elevatorVelocityTarget", elevatorVelocityTarget.inInchesPerSecond
    )
    CustomLogger.recordOutput("Elevator/elevatorVoltageTarget", elevatorVoltageTarget.inVolts)

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
        setVoltage(elevatorVoltageTarget)
        nextState = fromElevatorRequestToState(currentRequest)
      }
      ElevatorState.CLOSED_LOOP -> {

        io.setPosition(elevatorPositionTarget)

        nextState = fromElevatorRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }

  fun setVoltage(targetVoltage: ElectricalPotential) {
    if ((
      (upperLimitReached && targetVoltage > 0.0.volts) ||
        (lowerLimitReached && targetVoltage < 0.0.volts)
      ) && isHomed
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
