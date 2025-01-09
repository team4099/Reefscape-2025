package com.team4099.robot2025.subsystems.elevator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableNumber
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.perInch
import org.team4099.lib.units.derived.perInchSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.perSecond
import com.team4099.robot2025.subsystems.superstructure.Request.ElevatorRequest as ElevatorRequest

class Elevator(val io: ElevatorIO) {
  val inputs = ElevatorIO.ElevatorInputs()

  private val kP =
    LoggedTunableValue("Elevator/kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
  private val kI =
    LoggedTunableValue(
      "Elevator/kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "Elevator/kD", Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )

  private val slot1kP =
    LoggedTunableValue("Elevator/slot1kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
  private val slot1kI =
    LoggedTunableValue(
      "Elevator/slot1kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
    )
  private val slot1kD =
    LoggedTunableValue(
      "Elevator/slot1kD",
      Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )

  object TunableElevatorHeights {
    val enableElevator =
      LoggedTunableNumber(
        "Elevator/enableMovementElevator", if (ElevatorConstants.ENABLE_ELEVATOR) 1.0 else 0.0
      )

    val minPosition =
      LoggedTunableValue(
        "Elevator/minPosition",
        ElevatorConstants.DOWNWARDS_EXTENSION_LIMIT,
        Pair({ it.inInches }, { it.inches })
      )

    val maxPosition =
      LoggedTunableValue(
        "Elevator/maxPosition",
        ElevatorConstants.UPWARDS_EXTENSION_LIMIT,
        Pair({ it.inInches }, { it.inches })
      )

    val testPosition =
      LoggedTunableValue(
        "Elevator/testPosition",
        ElevatorConstants.L2_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val openLoopExtendVoltage =
      LoggedTunableValue(
        "Elevator/openLoopExtendVoltage",
        ElevatorConstants.OPEN_LOOP_EXTEND_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
    val openLoopRetractVoltage =
      LoggedTunableValue(
        "Elevator/openLoopRetractVoltage",
        ElevatorConstants.OPEN_LOOP_RETRACT_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val slamVelocity =
      LoggedTunableValue(
        "Elevator/slamVelocity",
        ElevatorConstants.SLAM_VELOCITY,
        Pair({ it.inInchesPerSecond }, { it.inches.perSecond })
      )

    val L1Height =
      LoggedTunableValue(
        "Elevator/L1Height", ElevatorConstants.L1_HEIGHT, Pair({ it.inInches }, { it.inches })
      )

    val L2Height =
      LoggedTunableValue(
        "Elevator/L2Height", ElevatorConstants.L2_HEIGHT, Pair({ it.inInches }, { it.inches })
      )

    val L3Height =
      LoggedTunableValue(
        "Elevator/L3Height", ElevatorConstants.L3_HEIGHT, Pair({ it.inInches }, { it.inches })
      )

    val L4Height =
      LoggedTunableValue(
        "Elevator/L4Height", ElevatorConstants.L4_HEIGHT, Pair({ it.inInches }, { it.inches })
      )

    val algaeLowHeight =
      LoggedTunableValue(
        "Elevator/algaeLowHeight",
        ElevatorConstants.ALGAE_LOW_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val algaeHighHeight =
      LoggedTunableValue(
        "Elevator/algaeHighHeight",
        ElevatorConstants.ALGAE_LOW_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val yPos =
      LoggedTunableValue(
        "Elevator/yPos",
        0.0.meters,
      )

    val y1Pos =
      LoggedTunableValue(
        "Elevator/y1Pos",
        0.0.meters,
      )
  }

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

  private var timeProfileGeneratedAt = Clock.fpgaTime
  private var lastHomingStatorCurrentTripTime = Clock.fpgaTime

  private var elevatorConstraints: TrapezoidProfile.Constraints<Meter> =
    TrapezoidProfile.Constraints(
      ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION
    )

  private val isAtTargetedPosition: Boolean
    get() =
      (
        currentRequest is ElevatorRequest.ClosedLoop &&
          (inputs.elevatorPosition - elevatorPositionTarget).absoluteValue <=
          ElevatorConstants.ELEVATOR_TOLERANCE
        ) ||
        (TunableElevatorHeights.enableElevator.get() != 1.0)

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
    TunableElevatorHeights

    if (RobotBase.isReal()) {
      isHomed = false

      kP.initDefault(ElevatorConstants.PID.REAL_KP)
      kI.initDefault(ElevatorConstants.PID.REAL_KI)
      kD.initDefault(ElevatorConstants.PID.REAL_KD)
    } else {
      isHomed = true

      kP.initDefault(ElevatorConstants.PID.SIM_KP)
      kI.initDefault(ElevatorConstants.PID.SIM_KI)
      kD.initDefault(ElevatorConstants.PID.SIM_KD)

      io.configFirstStagePID(kP.get(), kI.get(), kD.get())
      io.configSecondStagePID(slot1kP.get(), slot1kI.get(), slot1kD.get())
    }
  }

  fun periodic() {
    io.updateInputs(inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configFirstStagePID(kP.get(), kI.get(), kD.get())
    }
    if (slot1kD.hasChanged() || slot1kP.hasChanged() || slot1kI.hasChanged()) {
      io.configSecondStagePID(slot1kP.get(), slot1kI.get(), slot1kD.get())
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
