package com.team4099.lib.vision

import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.units.base.Time

data class TimestampedTrigVisionUpdate(
  val timestamp: Time,
  val robotTSpeaker: Transform2d,
)
