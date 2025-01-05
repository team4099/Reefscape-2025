package com.team4099.robot2023.config.constants

import com.ctre.phoenix.led.Animation
import com.ctre.phoenix.led.StrobeAnimation
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.volts

object LEDConstants {
  val INTAKE_CURRENT_THRESHOLD = 15.amps
  val OUTAKE_CURRENT_THRESHOLD = 20.amps
  val BATTERY_FULL_THRESHOLD = 12.5.volts
  val LED_COUNT = 50

  enum class CandleState(val animation: Animation?, val r: Int, val g: Int, val b: Int) {
    // Gold
    NO_NOTE(null, 0, 0, 0),
    NOTHING(null, 0, 0, 0),
    GOLD(null, 255, 105, 0),
    RED(null, 255, 0, 0),
    LIGHT_RED(null, 255, 67, 36),
    ORANGE(null, 255, 60, 0),
    BLUE(null, 0, 0, 255),
    PURPLE(null, 67, 36, 255),
    GREEN(null, 0, 255, 0),
    MAGENTA(null, 255, 0, 255),

    // Blue

    HAS_NOTE(null, 0, 0, 255),
    NO_TAG(null, 255, 0, 0),
    SEES_TAG(null, 255, 105, 0),
    SEES_NOTE(null, 255, 60, 0),

    // Yellow
    BATTERY_DISPLAY(null, 255, 105, 0),
    LOW_BATTERY_WARNING(null, 67, 36, 255),
    TUNING_MODE_WARNING(StrobeAnimation(255, 0, 0), 0, 0, 0),
    WHITE(null, 255, 255, 255),

    // Green

    CAN_SHOOT(null, 0, 255, 0)
  }
}
