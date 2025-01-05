package com.team4099.lib.phoenix6

import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotations
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerSecond
import org.team4099.lib.units.perSecond
import com.ctre.phoenix6.controls.PositionVoltage as PositionVoltagePhoenix6

class PositionVoltage(
  private var position:
    Angle, // Assuming an AngularPosition type exists similar to AngularVelocity
  private var enableFOC: Boolean = true,
  private var feedforward: ElectricalPotential = 0.0.volts,
  private var slot: Int = 0,
  private var overrideBrakeDurNeutral: Boolean = false,
  private var limitForwardMotion: Boolean = false,
  private var limitReverseMotion: Boolean = false,
  private val velocity: AngularVelocity = 0.0.degrees.perSecond
) {

  val positionVoltagePhoenix6 =
    PositionVoltagePhoenix6(
      position.inRotations,
    )
      .withVelocity(velocity.inRotationsPerSecond)
      .withEnableFOC(enableFOC)
      .withFeedForward(feedforward.inVolts)
      .withOverrideBrakeDurNeutral(overrideBrakeDurNeutral)
      .withLimitForwardMotion(limitForwardMotion)
      .withLimitReverseMotion(limitReverseMotion)

  fun setPosition(new_position: Angle) {
    position = new_position
    positionVoltagePhoenix6.Position = new_position.inRotations
  }

  fun setEnableFOC(new_enableFOC: Boolean) {
    enableFOC = new_enableFOC
    positionVoltagePhoenix6.EnableFOC = new_enableFOC
  }

  fun setFeedforward(new_feedforward: ElectricalPotential) {
    feedforward = new_feedforward
    positionVoltagePhoenix6.FeedForward = new_feedforward.inVolts
  }

  fun setSlot(new_slot: Int) {
    slot = new_slot
    positionVoltagePhoenix6.Slot = new_slot
  }

  fun setOverrideBrakeDurNeutral(new_override: Boolean) {
    overrideBrakeDurNeutral = new_override
    positionVoltagePhoenix6.OverrideBrakeDurNeutral = new_override
  }

  fun setLimitForwardMotion(new_limitForward: Boolean) {
    limitForwardMotion = new_limitForward
    positionVoltagePhoenix6.LimitForwardMotion = new_limitForward
  }

  fun setLimitReverseMotion(new_limitReverse: Boolean) {
    limitReverseMotion = new_limitReverse
    positionVoltagePhoenix6.LimitReverseMotion = new_limitReverse
  }
}
