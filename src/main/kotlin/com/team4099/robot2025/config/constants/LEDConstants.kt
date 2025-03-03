package com.team4099.robot2025.config.constants

import com.ctre.phoenix.led.Animation
import com.ctre.phoenix.led.StrobeAnimation
import org.team4099.lib.units.derived.volts

object LEDConstants {
  val BATTERY_FULL_THRESHOLD = 12.5.volts
  val LED_COUNT = 50

  enum class CandleState(val animation: Animation?, val r: Int, val g: Int, val b: Int) {
    NOTHING(null, 0, 0, 0),
    GOLD(null, 255, 105, 0),
    RED(null, 255, 0, 0),
    LIGHT_RED(null, 255, 67, 36),
    ORANGE(null, 255, 60, 0),
    BLUE(null, 0, 0, 255),
    PURPLE(null, 67, 36, 255),
    GREEN(null, 0, 255, 0),
    MAGENTA(null, 255, 0, 255),
    HAS_CORAL(null, 0, 0, 255),
    SEES_REEF_TAG(null, 255, 105, 0),
    NO_REEF_TAG(null, 255, 0, 0),
    IS_ALIGNING(null, 255, 105, 0),
    CAN_SCORE(null, 0, 255, 0),
    BATTERY_DISPLAY(null, 255, 105, 0),
    LOW_BATTERY_WARNING(null, 67, 36, 255),
    TUNING_MODE_WARNING(StrobeAnimation(255, 0, 0), 0, 0, 0),
    WHITE(null, 255, 255, 255),
  }
}
