package com.team4099.robot2023.subsystems.flywheel

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerRotations
import org.team4099.lib.units.derived.inVoltsPerRotationsPerMinute
import org.team4099.lib.units.derived.inVoltsPerRotationsPerMinutePerSecond
import org.team4099.lib.units.derived.perRadianPerSecond
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

class Flywheel(val io: FlywheelIO) : SubsystemBase() {

  object TunableFlywheelStates {
    val idleVelocity =
      LoggedTunableValue(
        "Flywheel/idleVelocity",
        FlywheelConstants.IDLE_VELOCITY,
        Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
      )
    val speakerVelocity =
      LoggedTunableValue(
        "Flywheel/speakerVelocity",
        FlywheelConstants.SPEAKER_VELOCITY,
        Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
      )
    val ampVelocity =
      LoggedTunableValue(
        "Flywheel/ampVelocity",
        FlywheelConstants.AMP_VELOCITY,
        Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
      )
    val trapVelocity =
      LoggedTunableValue(
        "Flywheel/trapVelocity",
        FlywheelConstants.TRAP_VELOCITY,
        Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
      )
    val ampScoreTime =
      LoggedTunableValue(
        "Flywheel/ampScoreTime",
        FlywheelConstants.AMP_SCORE_TIME,
        Pair({ it.inSeconds }, { it.seconds })
      )
    val speakerScoreTime =
      LoggedTunableValue(
        "Flywheel/speakerScoreTime",
        FlywheelConstants.SPEAKER_SCORE_TIME,
        Pair({ it.inSeconds }, { it.seconds })
      )
    val ejectVelocity =
      LoggedTunableValue(
        "Flywheel/ejectVelocity",
        FlywheelConstants.EJECT_VELOCITY,
        Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
      )

    val testVelocity =
      LoggedTunableValue(
        "Flywheel/testVelocity",
        FlywheelConstants.AMP_VELOCITY,
        Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
      )
    val passingShotVelocity =
      LoggedTunableValue(
        "Flywheel/passingShotVelocity",
        FlywheelConstants.PASSING_SHOT_VELOCITY,
        Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
      )

    val underStageShotVelocity =
      LoggedTunableValue(
        "Flywheel/underStageShotVelocity",
        FlywheelConstants.UNDER_STAGE_SHOT_VELOCITY,
        Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
      )
  }

  private val kP =
    LoggedTunableValue(
      "Flywheel/kP",
      Pair({ it.inVoltsPerRotationsPerMinute }, { it.volts / 1.0.rotations.perMinute })
    )
  private val kI =
    LoggedTunableValue(
      "Flywheel/kI",
      Pair({ it.inVoltsPerRotations }, { it.volts / (1.0.rotations.perMinute * 1.0.seconds) })
    )
  private val kD =
    LoggedTunableValue(
      "Flywheel/kD",
      Pair(
        { it.inVoltsPerRotationsPerMinutePerSecond },
        { it.volts / 1.0.rotations.perMinute.perSecond }
      )
    )

  val inputs = FlywheelIO.FlywheelIOInputs()
  private val flywheelkS = LoggedTunableValue("Flywheel/kS", Pair({ it.inVolts }, { it.volts }))
  private val flywheelkV =
    LoggedTunableValue(
      "Flywheel/kV", Pair({ it.inVoltsPerRadianPerSecond }, { it.volts.perRadianPerSecond })
    )
  private val flywheelkA =
    LoggedTunableValue(
      "Flywheel/kA",
      Pair(
        { it.inVoltsPerRadianPerSecondPerSecond },
        { it.volts / 1.0.radians.perSecond.perSecond }
      )
    )

  private val flywheelTestVelocity =
    LoggedTunableValue(
      "Flywheel/testVelocity", Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
    )

  var flywheelFeedForward: SimpleMotorFeedforward<Radian, Volt>

  var lastFlywheelRunTime = 0.0.seconds
  private var lastRightFlywheelVoltage = 0.0.volts

  var flywheelTargetVoltage = 0.volts
  var flywheelRightTargetVelocity: AngularVelocity = -1337.0.rotations.perMinute
  var flywheelLeftTargetVelocity: AngularVelocity = -1337.0.rotations.perMinute

  val isAtTargetedVelocity: Boolean
    get() =
      (
        currentState == FlywheelStates.TARGETING_VELOCITY &&
          (inputs.leftFlywheelVelocity - flywheelLeftTargetVelocity).absoluteValue <=
          FlywheelConstants.FLYWHEEL_TOLERANCE
        ) || inputs.isSimulated

  var currentState = Companion.FlywheelStates.UNINITIALIZED

  var currentRequest: Request.FlywheelRequest = Request.FlywheelRequest.OpenLoop(0.0.volts)
    set(value) {
      when (value) {
        is Request.FlywheelRequest.OpenLoop -> {
          flywheelTargetVoltage = value.flywheelVoltage
        }
        is Request.FlywheelRequest.TargetingVelocity -> {
          flywheelRightTargetVelocity = value.flywheelVelocity * 2
          // left needs to be half of the right one
          flywheelLeftTargetVelocity = value.flywheelVelocity
        }
      }
      field = value
    }

  init {
    if (RobotBase.isReal()) {
      kP.initDefault(FlywheelConstants.PID.REAL_KP)
      kI.initDefault(FlywheelConstants.PID.REAL_KI)
      kD.initDefault(FlywheelConstants.PID.REAL_KD)

      flywheelTestVelocity.initDefault(1000.rotations.perMinute)

      flywheelFeedForward =
        SimpleMotorFeedforward(
          FlywheelConstants.PID.REAL_FLYWHEEL_KS,
          FlywheelConstants.PID.REAL_FLYWHEEL_KV,
          FlywheelConstants.PID.REAL_FLYWHEEL_KA
        )

      flywheelkS.initDefault(FlywheelConstants.PID.REAL_FLYWHEEL_KS)
      flywheelkV.initDefault(FlywheelConstants.PID.REAL_FLYWHEEL_KV)
      flywheelkA.initDefault(FlywheelConstants.PID.REAL_FLYWHEEL_KA)
    } else {
      kP.initDefault(FlywheelConstants.PID.SIM_KP)
      kI.initDefault(FlywheelConstants.PID.SIM_KI)
      kD.initDefault(FlywheelConstants.PID.SIM_KD)

      flywheelkS.initDefault(FlywheelConstants.PID.SIM_FLYWHEEL_KS)
      flywheelkV.initDefault(FlywheelConstants.PID.SIM_FLYWHEEL_KV)
      flywheelkA.initDefault(FlywheelConstants.PID.SIM_FLYWHEEL_KA)

      flywheelFeedForward =
        SimpleMotorFeedforward(
          FlywheelConstants.PID.SIM_FLYWHEEL_KS,
          FlywheelConstants.PID.SIM_FLYWHEEL_KV,
          FlywheelConstants.PID.SIM_FLYWHEEL_KA
        )
    }

    io.configPID(kP.get(), kI.get(), kD.get(), flywheelkV.get())
  }
  override fun periodic() {
    io.updateInputs(inputs)

    Logger.processInputs("Flywheel", inputs)
    Logger.recordOutput(
      "Flywheel/targetDifference",
      (inputs.leftFlywheelVelocity - flywheelLeftTargetVelocity)
        .absoluteValue
        .inRotationsPerMinute
    )
    Logger.recordOutput("Flywheel/currentState", currentState.name)

    Logger.recordOutput("Flywheel/requestedState", currentRequest.javaClass.simpleName)

    Logger.recordOutput("Flywheel/isAtTargetedVelocity", isAtTargetedVelocity)

    Logger.recordOutput("Flywheel/FlywheelRightTargetVoltage", flywheelTargetVoltage.inVolts)
    Logger.recordOutput("Flywheel/FlywheelLeftTargetVoltage", flywheelTargetVoltage.inVolts)

    Logger.recordOutput(
      "Flywheel/FlywheelRightTargetVelocity", flywheelRightTargetVelocity.inRotationsPerMinute
    )
    Logger.recordOutput(
      "Flywheel/FlywheelLeftTargetVelocity", flywheelLeftTargetVelocity.inRotationsPerMinute
    )
    if (Constants.Tuning.DEBUGING_MODE) {

      Logger.recordOutput("Flywheel/FlywheelLastVoltage", lastRightFlywheelVoltage.inVolts)
    }

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || flywheelkV.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get(), flywheelkV.get())
    }

    if (flywheelkA.hasChanged() || flywheelkV.hasChanged() || flywheelkS.hasChanged()) {
      flywheelFeedForward =
        SimpleMotorFeedforward(flywheelkS.get(), flywheelkV.get(), flywheelkA.get())
    }

    var nextState = currentState
    when (currentState) {
      Companion.FlywheelStates.UNINITIALIZED -> {
        nextState = Companion.FlywheelStates.OPEN_LOOP
      }
      Companion.FlywheelStates.OPEN_LOOP -> {
        setFlywheelVoltage(flywheelTargetVoltage)
        lastFlywheelRunTime = Clock.fpgaTime

        nextState = fromRequestToState(currentRequest)
      }
      Companion.FlywheelStates.TARGETING_VELOCITY -> {
        setFlywheelVelocity(flywheelLeftTargetVelocity)
        lastFlywheelRunTime = Clock.fpgaTime
        nextState = fromRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }

  fun setFlywheelVoltage(appliedVoltage: ElectricalPotential) {
    io.setFlywheelVoltage(appliedVoltage)
  }

  fun setFlywheelVelocity(
    velocity: AngularVelocity,
  ) {
    val feedforward = flywheelFeedForward.calculate(velocity)

    Logger.recordOutput("Flywheel/FeedForward", feedforward.inVolts)

    io.setFlywheelVelocity(velocity, feedforward)
  }

  fun flywheelSpinUpCommand(): Command {
    return runOnce {
      currentRequest = Request.FlywheelRequest.TargetingVelocity(flywheelTestVelocity.get())
    }
  }

  fun flywheelStopCommand(): Command {
    return runOnce {
      currentRequest = Request.FlywheelRequest.TargetingVelocity(0.rotations.perMinute)
    }
  }

  fun flywheelOpenLoopCommand(): Command {
    return runOnce({ currentRequest = Request.FlywheelRequest.OpenLoop(3.volts) })
  }

  fun flywheelResetCommand(): Command {
    return runOnce({ currentRequest = Request.FlywheelRequest.OpenLoop(0.volts) })
  }

  companion object {
    enum class FlywheelStates {
      UNINITIALIZED,
      OPEN_LOOP,
      TARGETING_VELOCITY;
      fun equivalentToRequest(request: Request.FlywheelRequest): Boolean {
        return (request is Request.FlywheelRequest.OpenLoop && this == OPEN_LOOP) ||
          (request is Request.FlywheelRequest.TargetingVelocity && this == TARGETING_VELOCITY)
      }
    }
    inline fun fromRequestToState(request: Request.FlywheelRequest): FlywheelStates {
      return when (request) {
        is Request.FlywheelRequest.OpenLoop -> FlywheelStates.OPEN_LOOP
        is Request.FlywheelRequest.TargetingVelocity -> FlywheelStates.TARGETING_VELOCITY
      }
    }
  }
}
