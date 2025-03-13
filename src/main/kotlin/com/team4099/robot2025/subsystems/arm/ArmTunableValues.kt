package com.team4099.robot2025.subsystems.arm

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.ArmConstants
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.volts

object ArmTunableValues {
  val armKP = LoggedTunableValue("Arm/armKP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))

  val armKI =
    LoggedTunableValue(
      "Arm/armKI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )

  val armKD =
    LoggedTunableValue(
      "Arm/armKD", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  object ArmVoltages {
    val idleVoltage =
      LoggedTunableValue(
        "Arm/idleVoltage", ArmConstants.IDLE_VOLTAGE, Pair({ it.inVolts }, { it.volts })
      )

    val retractVoltage =
      LoggedTunableValue(
        "Arm/retractVoltage", ArmConstants.RETRACT_VOLTAGE, Pair({ it.inVolts }, { it.volts })
      )

    val extendVoltage =
      LoggedTunableValue(
        "Arm/extendVoltage", ArmConstants.EXTEND_VOLTAGE, Pair({ it.inVolts }, { it.volts })
      )
  }
}
