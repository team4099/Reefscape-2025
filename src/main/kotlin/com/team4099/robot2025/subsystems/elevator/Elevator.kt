package com.team4099.robot2023.subsystems.elevator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableNumber
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ElevatorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecondPerSecond
import org.team4099.lib.units.derived.perInch
import org.team4099.lib.units.derived.perInchSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.perSecond
import com.team4099.robot2023.subsystems.superstructure.Request.ElevatorRequest as ElevatorRequest

class Elevator(val io: ElevatorIO) : SubsystemBase() {
  val inputs = ElevatorIO.ElevatorInputs()
  private var elevatorFeedforward: ElevatorFeedforward =
    ElevatorFeedforward(
      ElevatorConstants.ELEVATOR_KS,
      ElevatorConstants.ELEVATOR_KG,
      ElevatorConstants.ELEVATOR_KV,
      ElevatorConstants.ELEVATOR_KA
    )

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
  private val kS = LoggedTunableValue("Elevator/kS", Pair({ it.inVolts }, { it.volts }))
  private val kG = LoggedTunableValue("Elevator/kG", Pair({ it.inVolts }, { it.volts }))
  private val kV =
    LoggedTunableValue(
      "Elevator/kV", Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )
  private val kA =
    LoggedTunableValue(
      "Elevator/kA",
      Pair(
        { it.inVoltsPerMeterPerSecondPerSecond },
        { it.volts / 1.0.meters.perSecond.perSecond }
      )
    )

  object TunableElevatorHeights {
    val enableElevator =
      LoggedTunableNumber(
        "Elevator/enableMovementElevator", if (ElevatorConstants.ENABLE_ELEVATOR) 1.0 else 0.0
      )

    val minPosition =
      LoggedTunableValue(
        "Elevator/minPosition",
        ElevatorConstants.ELEVATOR_IDLE_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )
    val maxPosition =
      LoggedTunableValue(
        "Elevator/maxPosition",
        ElevatorConstants.ELEVATOR_SOFT_LIMIT_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )
    val testPosition =
      LoggedTunableValue(
        "Elevator/testPosition",
        ElevatorConstants.SHOOT_SPEAKER_MID_POSITION,
        Pair({ it.inInches }, { it.inches })
      )
    val climbExtend =
      LoggedTunableValue(
        "Elevator/climbExtend",
        ElevatorConstants.ELEVATOR_CLIMB_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    // TODO: change voltages
    val openLoopExtendVoltage =
      LoggedTunableValue(
        "Elevator/openLoopExtendVoltage",
        ElevatorConstants.ELEVATOR_OPEN_LOOP_EXTEND_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
    val openLoopRetractVoltage =
      LoggedTunableValue(
        "Elevator/openLoopRetractVoltage",
        ElevatorConstants.ELEVATOR_OPEN_LOOP_RETRACT_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val shootSpeakerLow =
      LoggedTunableValue(
        "Elevator/shootSpeakerLow",
        ElevatorConstants.SHOOT_SPEAKER_LOW_POSITION,
        Pair({ it.inInches }, { it.inches })
      )
    val shootSpeakerMid =
      LoggedTunableValue(
        "Elevator/shootSpeakerMid",
        ElevatorConstants.SHOOT_SPEAKER_MID_POSITION,
        Pair({ it.inInches }, { it.inches })
      )
    val shootSpeakerHigh =
      LoggedTunableValue(
        "Elevator/shootSpeakerHigh",
        ElevatorConstants.SHOOT_SPEAKER_HIGH_POSITION,
        Pair({ it.inInches }, { it.inches })
      )
    val shootAmpFeederPosition =
      LoggedTunableValue(
        "Elevator/shootAmpFeederPosition",
        ElevatorConstants.SHOOT_AMP_FEEDER_SIDE_POSITION,
        Pair({ it.inInches }, { it.inches })
      )

    val shootAmpShooterPosition =
      LoggedTunableValue(
        "Elevator/shootAmpShooterPosition",
        ElevatorConstants.SHOOT_AMP_SHOOTER_SIDE_POSITION,
        Pair({ it.inInches }, { it.inches })
      )

    val sourceNoteOffset =
      LoggedTunableValue(
        "Elevator/sourceNoteOffset",
        ElevatorConstants.SOURCE_NOTE_OFFSET,
        Pair({ it.inInches }, { it.inches })
      )
  }

  val forwardLimitReached: Boolean
    get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_SOFT_LIMIT_EXTENSION
  val reverseLimitReached: Boolean
    get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_SOFT_LIMIT_RETRACTION

  val forwardOpenLoopLimitReached: Boolean
    get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_OPEN_LOOP_SOFT_LIMIT_EXTENSION
  val reverseOpenLoopLimitReached: Boolean
    get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_OPEN_LOOP_SOFT_LIMIT_RETRACTION

  var isHomed = false

  var currentState: ElevatorState = ElevatorState.UNINITIALIZED
  var currentRequest: ElevatorRequest = ElevatorRequest.OpenLoop(0.0.volts)
    set(value) {
      when (value) {
        is ElevatorRequest.OpenLoop -> elevatorVoltageTarget = value.voltage
        is ElevatorRequest.TargetingPosition -> {
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

  private var lastRequestedPosition = -9999.inches
  private var lastRequestedVelocity = -9999.inches.perSecond
  private var lastRequestedVoltage = -9999.volts

  private var timeProfileGeneratedAt = Clock.fpgaTime

  private var lastHomingStatorCurrentTripTime = Clock.fpgaTime

  // Trapezoidal Profile Constraints
  private var elevatorConstraints: TrapezoidProfile.Constraints<Meter> =
    TrapezoidProfile.Constraints(
      ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION
    )
  private var elevatorSetpoint: TrapezoidProfile.State<Meter> =
    TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)
  private var elevatorProfile =
    TrapezoidProfile(
      elevatorConstraints,
      TrapezoidProfile.State(-9999.inches, -9999.inches.perSecond),
      TrapezoidProfile.State(-9999.inches, -9999.inches.perSecond)
    )

  val isAtTargetedPosition: Boolean
    get() =
      (
        currentRequest is ElevatorRequest.TargetingPosition &&
          elevatorProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
          (inputs.elevatorPosition - elevatorPositionTarget).absoluteValue <=
          ElevatorConstants.ELEVATOR_TOLERANCE
        ) ||
        (TunableElevatorHeights.enableElevator.get() != 1.0)

  val canContinueSafely: Boolean
    get() =
      currentRequest is ElevatorRequest.TargetingPosition &&
        (
          (
            (inputs.elevatorPosition - elevatorPositionTarget).absoluteValue <=
              ElevatorConstants.ELEVATOR_SAFE_THRESHOLD
            ) ||
            elevatorProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt)
          ) &&
        lastRequestedPosition == elevatorPositionTarget

  init {
    TunableElevatorHeights

    // Initializes PID constants and changes FF depending on if sim or real
    if (RobotBase.isReal()) {
      isHomed = false

      kP.initDefault(ElevatorConstants.REAL_KP)
      kI.initDefault(ElevatorConstants.REAL_KI)
      kD.initDefault(ElevatorConstants.REAL_KD)
    } else {
      isHomed = true

      kP.initDefault(ElevatorConstants.SIM_KP)
      kI.initDefault(ElevatorConstants.SIM_KI)
      kD.initDefault(ElevatorConstants.SIM_KD)

      io.configPID(kP.get(), kI.get(), kD.get())
    }

    kV.initDefault(ElevatorConstants.ELEVATOR_KV)
    kS.initDefault(ElevatorConstants.ELEVATOR_KS)
    kG.initDefault(ElevatorConstants.ELEVATOR_KG)
    kA.initDefault(ElevatorConstants.ELEVATOR_KA)

    elevatorFeedforward =
      ElevatorFeedforward(
        ElevatorConstants.ELEVATOR_KS,
        ElevatorConstants.ELEVATOR_KG,
        ElevatorConstants.ELEVATOR_KV,
        ElevatorConstants.ELEVATOR_KA
      )
  }

  override fun periodic() {
    io.updateInputs(inputs)
    if ((kP.hasChanged()) || (kI.hasChanged()) || (kD.hasChanged())) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    if (kS.hasChanged() || kV.hasChanged() || kA.hasChanged() || kG.hasChanged()) {
      elevatorFeedforward = ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get())
    }
    Logger.processInputs("Elevator", inputs)
    Logger.recordOutput("Elevator/currentState", currentState.name)
    Logger.recordOutput("Elevator/currentRequest", currentRequest.javaClass.simpleName)
    Logger.recordOutput(
      "Elevator/elevatorHeight",
      (inputs.elevatorPosition - ElevatorConstants.ELEVATOR_GROUND_OFFSET).inInches
    )
    Logger.recordOutput("Elevator/requestedPosition", elevatorPositionTarget.inInches)

    if (Constants.Tuning.DEBUGING_MODE) {
      Logger.recordOutput("Elevator/isHomed", isHomed)
      Logger.recordOutput("Elevator/canContinueSafely", canContinueSafely)

      Logger.recordOutput("Elevator/isAtTargetPosition", isAtTargetedPosition)
      Logger.recordOutput("Elevator/lastGeneratedAt", timeProfileGeneratedAt.inSeconds)

      Logger.recordOutput("Elevator/elevatorPositionTarget", elevatorPositionTarget.inInches)
      Logger.recordOutput(
        "Elevator/elevatorVelocityTarget", elevatorVelocityTarget.inInchesPerSecond
      )
      Logger.recordOutput("Elevator/elevatorVoltageTarget", elevatorVoltageTarget.inVolts)

      Logger.recordOutput("Elevator/lastElevatorPositionTarget", lastRequestedPosition.inInches)
      Logger.recordOutput(
        "Elevator/lastElevatorVelocityTarget", lastRequestedVelocity.inInchesPerSecond
      )
      Logger.recordOutput("Elevator/lastElevatorVoltageTarget", lastRequestedVoltage.inVolts)

      Logger.recordOutput("Elevator/forwardLimitReached", forwardLimitReached)
      Logger.recordOutput("Elevator/reverseLimitReached", reverseLimitReached)
    }
    var nextState = currentState
    when (currentState) {
      ElevatorState.UNINITIALIZED -> {
        nextState = fromElevatorRequestToState(currentRequest)
        zeroEncoder()
      }
      ElevatorState.OPEN_LOOP -> {
        setOutputVoltage(elevatorVoltageTarget)
        nextState = fromElevatorRequestToState(currentRequest)
      }
      ElevatorState.TARGETING_POSITION -> {
        if ((elevatorPositionTarget != lastRequestedPosition) ||
          (elevatorVelocityTarget != lastRequestedVelocity)
        ) {
          val preProfileGenerate = Clock.realTimestamp
          elevatorProfile =
            TrapezoidProfile(
              elevatorConstraints,
              TrapezoidProfile.State(elevatorPositionTarget, elevatorVelocityTarget),
              TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)
            )
          val postProfileGenerate = Clock.realTimestamp
          Logger.recordOutput(
            "/Elevator/profileGenerationMS",
            postProfileGenerate.inSeconds - preProfileGenerate.inSeconds
          )
          Logger.recordOutput("Elevator/initialPosition", elevatorProfile.initial.position.inInches)
          Logger.recordOutput(
            "Elevator/initialVelocity", elevatorProfile.initial.velocity.inInchesPerSecond
          )
          timeProfileGeneratedAt = Clock.fpgaTime
          lastRequestedPosition = elevatorPositionTarget
          lastRequestedVelocity = elevatorVelocityTarget
        }
        val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt
        val profilePosition = elevatorProfile.calculate(timeElapsed)
        setPosition(profilePosition)
        Logger.recordOutput(
          "Elevator/completedMotionProfile", elevatorProfile.isFinished(timeElapsed)
        )
        Logger.recordOutput(
          "Elevator/profileTargetVelocity", profilePosition.velocity.inInchesPerSecond
        )
        Logger.recordOutput("Elevator/profileTargetPosition", profilePosition.position.inInches)
        Logger.recordOutput("Elevator/timeElapsed", timeElapsed.inSeconds)
        nextState = fromElevatorRequestToState(currentRequest)
        if (!(currentState.equivalentToRequest(currentRequest))) {
          lastRequestedVelocity = -999.inches.perSecond
          lastRequestedPosition = -999.inches
        }
      }
      ElevatorState.HOME -> {
        zeroEncoder()
        isHomed = true

        if (isHomed) {
          nextState = fromElevatorRequestToState(currentRequest)
        }
        /*

        if (inputs.leaderStatorCurrent < ElevatorConstants.HOMING_STATOR_CURRENT || inputs.followerStatorCurrent < ElevatorConstants.HOMING_STATOR_CURRENT ) {
          lastHomingStatorCurrentTripTime = Clock.fpgaTime
        }
        if (!inputs.isSimulating &&
          (
            !isHomed &&
                    (inputs.leaderStatorCurrent < ElevatorConstants.HOMING_STATOR_CURRENT || inputs.followerStatorCurrent < ElevatorConstants.HOMING_STATOR_CURRENT ) &&
              Clock.fpgaTime - lastHomingStatorCurrentTripTime <
              ElevatorConstants.HOMING_STALL_TIME_THRESHOLD
            )
        ) {
          io.setOutputVoltage(ElevatorConstants.HOMING_APPLIED_VOLTAGE)
        } else {
          zeroEncoder()
          isHomed = true
        }

         */
      }
    }
    currentState = nextState
  }

  fun setOutputVoltage(voltage: ElectricalPotential) {
    if ((forwardLimitReached) && (voltage > 0.volts) ||
      (reverseLimitReached) && (voltage < 0.volts)
    ) {
      io.setOutputVoltage(0.volts)
    } else {
      io.setOutputVoltage(voltage)
    }
  }

  fun setHomeVoltage(voltage: ElectricalPotential) {
    io.setOutputVoltage(voltage)
  }

  fun zeroEncoder() {
    io.zeroEncoder()
  }

  fun setPosition(setpoint: TrapezoidProfile.State<Meter>) {
    val elevatorAcceleration =
      (setpoint.velocity - elevatorSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME
    elevatorSetpoint = setpoint
    val feedForward = elevatorFeedforward.calculate(setpoint.velocity, elevatorAcceleration)
    if ((forwardLimitReached) && (setpoint.position > inputs.elevatorPosition) ||
      (reverseLimitReached) && (setpoint.position < inputs.elevatorPosition)
    ) {
      io.setPosition(inputs.elevatorPosition, feedForward)
    } else {
      io.setPosition(setpoint.position, feedForward)
    }
  }

  companion object {
    enum class ElevatorState {
      UNINITIALIZED,
      TARGETING_POSITION,
      OPEN_LOOP,
      HOME;
      inline fun equivalentToRequest(request: ElevatorRequest): Boolean {
        return (request is ElevatorRequest.Home && this == HOME) ||
          (request is ElevatorRequest.OpenLoop && this == OPEN_LOOP) ||
          (request is ElevatorRequest.TargetingPosition && this == TARGETING_POSITION)
      }
    }

    inline fun fromElevatorRequestToState(request: ElevatorRequest): ElevatorState {
      return when (request) {
        is ElevatorRequest.Home -> ElevatorState.HOME
        is ElevatorRequest.OpenLoop -> ElevatorState.OPEN_LOOP
        is ElevatorRequest.TargetingPosition -> ElevatorState.TARGETING_POSITION
      }
    }
  }

  fun testElevatorOpenLoopRetractCommand(): Command {
    return runOnce({ currentRequest = ElevatorRequest.OpenLoop(-10.volts) })
  }

  fun testElevatorOpenLoopExtendCommand(): Command {
    return runOnce({ currentRequest = ElevatorRequest.OpenLoop(10.volts) })
  }

  fun elevatorClosedLoopRetractCommand(): Command {
    return runOnce({ currentRequest = ElevatorRequest.TargetingPosition(2.inches) })
  }

  fun testElevatorClosedLoopExtendCommand(): Command {
    return runOnce({ currentRequest = ElevatorRequest.TargetingPosition(10.inches) })
  }
}
