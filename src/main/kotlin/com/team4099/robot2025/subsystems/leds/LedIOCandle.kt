package com.team4099.robot2025.subsystems.led

import com.ctre.phoenix.led.CANdle
import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.LEDConstants
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inSeconds
import kotlin.math.absoluteValue

object LedIOCandle : LedIO {

  private val ledController = CANdle(Constants.LED.LED_CANDLE_ID)
  private var lastState: LEDConstants.CandleState = LEDConstants.CandleState.NOTHING
  private var waveRuns = 0
  private var loopCycles = 0
  private var reverseLEDS = false
  private var finishedFade = false

  override fun updateInputs(inputs: LedIO.LedIOInputs) {
    inputs.ledState = lastState.name
  }

  override fun setState(newState: LEDConstants.CandleState) {
    Logger.recordOutput("LED/newState", newState)
    lastState = newState
    setCANdleState(newState)
  }

  private fun wave(
    state: LEDConstants.CandleState,
    defaultState: LEDConstants.CandleState,
    lengthOfWave: Int = 10
  ) {
    ledController.setLEDs(state.r, state.g, state.b, 0, waveRuns, lengthOfWave)

    // Set outer colors
    ledController.setLEDs(defaultState.r, defaultState.g, defaultState.b, 0, 0, waveRuns)
    ledController.setLEDs(
      defaultState.r,
      defaultState.g,
      defaultState.b,
      0,
      waveRuns + lengthOfWave,
      LEDConstants.LED_COUNT - waveRuns - lengthOfWave
    )

    if (waveRuns >= LEDConstants.LED_COUNT - lengthOfWave) {
      reverseLEDS = true
    } else if (waveRuns < lengthOfWave) {
      reverseLEDS = false
    }

    waveRuns += if (!reverseLEDS) 1 else -1
  }

  private fun gradient(
    state: LEDConstants.CandleState,
    otherState: LEDConstants.CandleState,
    lengthOfSolidEnds: Int = 10
  ) {
    ledController.setLEDs(state.r, state.g, state.b, 0, 0, lengthOfSolidEnds)
    ledController.setLEDs(
      otherState.r,
      otherState.g,
      otherState.b,
      0,
      LEDConstants.LED_COUNT - lengthOfSolidEnds,
      lengthOfSolidEnds
    )

    for (
      (step, ledIdx) in
      (lengthOfSolidEnds until (LEDConstants.LED_COUNT - lengthOfSolidEnds)).withIndex()
    ) {
      ledController.setLEDs(
        (
          state.r +
            (otherState.r - state.r) *
            ((step + 1) / (LEDConstants.LED_COUNT - lengthOfSolidEnds * 2.0))
          )
          .toInt(),
        (
          state.g +
            (otherState.g - state.g) *
            ((step + 1) / (LEDConstants.LED_COUNT - lengthOfSolidEnds * 2.0))
          )
          .toInt(),
        (
          state.b +
            (otherState.b - state.b) *
            ((step + 1) / (LEDConstants.LED_COUNT - lengthOfSolidEnds * 2.0))
          )
          .toInt(),
        0,
        ledIdx,
        1
      )
    }
  }

  private fun progressBar(
    state: LEDConstants.CandleState,
    defaultState: LEDConstants.CandleState,
    percent: Double
  ) {
    ledController.setLEDs(
      state.r, state.g, state.b, 0, 0, (percent * LEDConstants.LED_COUNT).toInt()
    )
    ledController.setLEDs(
      defaultState.r,
      defaultState.g,
      defaultState.b,
      0,
      (percent * LEDConstants.LED_COUNT).toInt(),
      LEDConstants.LED_COUNT
    )
  }

  private fun fadeBetweenColors(
    state: LEDConstants.CandleState,
    otherState: LEDConstants.CandleState,
    loopCyclesToConverge: Int = 10
  ) {
    val stepUpInLoopCycles = 2
    var reachedStartColor = true
    var reachedEndColor = true

    for (
      (ledIdx, numberOfLoopCycles) in
      (
        loopCyclesToConverge..loopCyclesToConverge +
          LEDConstants.LED_COUNT * stepUpInLoopCycles step stepUpInLoopCycles
        )
        .withIndex()
    ) {
      val calculatedR =
        (
          state.r +
            (otherState.r - state.r) *
            (
              if (loopCycles > numberOfLoopCycles) 1.0
              else (loopCycles / numberOfLoopCycles.toDouble())
              )
          )
          .toInt()
      val calculatedG =
        (
          state.g +
            (otherState.g - state.g) *
            (
              if (loopCycles > numberOfLoopCycles) 1.0
              else (loopCycles / numberOfLoopCycles.toDouble())
              )
          )
          .toInt()
      val calculatedB =
        (
          state.b +
            (otherState.b - state.b) *
            (
              if (loopCycles > numberOfLoopCycles) 1.0
              else (loopCycles / numberOfLoopCycles.toDouble())
              )
          )
          .toInt()

      reachedStartColor =
        (calculatedR == state.r && calculatedG == state.g && calculatedB == state.b)
      reachedEndColor =
        (
          (calculatedR - otherState.r).absoluteValue < 5 &&
            (calculatedG - otherState.g).absoluteValue < 5 &&
            (calculatedB - otherState.b).absoluteValue < 5
          )

      ledController.setLEDs(calculatedR, calculatedG, calculatedB, 0, ledIdx, 1)
    }

    loopCycles += if (finishedFade) -1 else 1

    if (reachedStartColor) {
      finishedFade = false
    }

    if (reachedEndColor) { // All LEDs are at the end color
      finishedFade = true
    }
  }

  private fun hsvToRgb(h: Float, s: Float, v: Float): Triple<Int, Int, Int> {
    if (s <= 0.0f) {
      val gray = (v * 255).toInt()
      return Triple(gray, gray, gray)
    }

    val hh = (if (h >= 360f) 0f else h) / 60f
    val i = hh.toInt()
    val ff = hh - i

    val p = v * (1.0f - s)
    val q = v * (1.0f - (s * ff))
    val t = v * (1.0f - (s * (1.0f - ff)))

    val (r, g, b) =
      when (i) {
        0 -> Triple(v, t, p)
        1 -> Triple(q, v, p)
        2 -> Triple(p, v, t)
        3 -> Triple(p, q, v)
        4 -> Triple(t, p, v)
        else -> Triple(v, p, q)
      }

    return Triple((r * 255).toInt(), (g * 255).toInt(), (b * 255).toInt())
  }

  private fun rainbow() {
    // Calculate how far through the cycle we are (0.0 to 1.0)
    val cycleProgress = (Clock.fpgaTime.inSeconds % 3) / 3

    // Update each light with an offset based on its position
    for (i in 0 until LEDConstants.LED_COUNT) {
      // Calculate the phase for this light (spread evenly across the cycle)
      val position = (cycleProgress + (i.toDouble() / 60)) % 1f
      val hue = position * 360f
      val (r, g, b) = hsvToRgb(hue.toFloat(), 1.0f, 1.0f)

      ledController.setLEDs(r, g, b, 0, i, 1)
    }
  }

  private fun setCANdleState(state: LEDConstants.CandleState) {
    if (state == LEDConstants.CandleState.LOW_BATTERY_WARNING) {
      wave(state, LEDConstants.CandleState.NOTHING)
    } else if (state == LEDConstants.CandleState.IS_ALIGNING) {
      ledController.clearAnimation(0)
      rainbow()
    } else if (state.animation == null) {
      ledController.clearAnimation(0)
      ledController.setLEDs(state.r, state.g, state.b)
    } else {
      ledController.animate(state.animation, 0)
      ledController.setLEDs(state.r, state.g, state.b)
    }
  }
}
