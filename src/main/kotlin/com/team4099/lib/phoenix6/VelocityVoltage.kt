package com.team4099.lib.phoenix6

import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerSecond
import org.team4099.lib.units.inRotationsPerSecondPerSecond
import org.team4099.lib.units.perSecond
import com.ctre.phoenix6.controls.VelocityVoltage as VelocityVoltagePhoenix6

class VelocityVoltage(
  private var velocity: AngularVelocity,
  private var acceleration: AngularAcceleration = 0.0.degrees.perSecond.perSecond,
  private var enableFOC: Boolean = true,
  private var feedforward: ElectricalPotential = 0.0.volts,
  private var slot: Int = 0,
  private var overrideBrakeDurNeutral: Boolean = false,
  private var limitForwardMotion: Boolean = false,
  private var limitReverseMotion: Boolean = false
) {

  val velocityVoltagePhoenix6 =
    VelocityVoltagePhoenix6(
      velocity.inRotationsPerSecond,
    )
      .withAcceleration(acceleration.inRotationsPerSecondPerSecond)
      .withEnableFOC(enableFOC)
      .withFeedForward(feedforward.inVolts)
      .withSlot(slot)
      .withOverrideBrakeDurNeutral(overrideBrakeDurNeutral)
      .withLimitForwardMotion(limitForwardMotion)
      .withLimitReverseMotion(limitReverseMotion)

  fun setVelocity(new_velocity: AngularVelocity) {
    velocity = new_velocity
    velocityVoltagePhoenix6.Velocity = velocity.inRotationsPerSecond
  }

  fun setAcceleration(new_accel: AngularAcceleration) {
    acceleration = new_accel
    velocityVoltagePhoenix6.Acceleration = acceleration.inRotationsPerSecondPerSecond
  }

  fun setEnableFOC(new_enableFOC: Boolean) {
    enableFOC = new_enableFOC
    velocityVoltagePhoenix6.EnableFOC = new_enableFOC
  }

  fun setFeedforward(new_feedforward: ElectricalPotential) {
    feedforward = new_feedforward
    velocityVoltagePhoenix6.FeedForward = new_feedforward.inVolts
  }

  fun setSlot(new_slot: Int) {
    slot = new_slot
    velocityVoltagePhoenix6.Slot = new_slot
  }

  fun setOverrideBrakeDurNeutral(new_override: Boolean) {
    overrideBrakeDurNeutral = new_override
    velocityVoltagePhoenix6.OverrideBrakeDurNeutral = new_override
  }

  fun setLimitForwardMotion(new_limitForward: Boolean) {
    limitForwardMotion = new_limitForward
    velocityVoltagePhoenix6.LimitForwardMotion = new_limitForward
  }

  fun setLimitReverseMotion(new_limitReverse: Boolean) {
    limitReverseMotion = new_limitReverse
    velocityVoltagePhoenix6.LimitReverseMotion = new_limitReverse
  }
}
