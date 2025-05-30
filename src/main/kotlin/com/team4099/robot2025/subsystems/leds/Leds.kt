package com.team4099.robot2023.subsystems.led

import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.LEDConstants
import com.team4099.robot2025.subsystems.led.LedIO
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.FMSData
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.volts

class Leds(val io: LedIO) {
  var inputs = LedIO.LedIOInputs()

  var hasCoral = false
  var isAutoAligning = false
  var isAligned = true
  var seesTag = true
  var closestReefTagID = -1
  var isIntaking = false
  var isTuningDebugging = Constants.Tuning.TUNING_MODE || Constants.Tuning.DEBUGING_MODE

  var state = LEDConstants.CandleState.NOTHING
    set(value) {
      io.setState(value)
      field = value
    }

  fun ledColorBasedOnTag(): LEDConstants.CandleState {
    if (DriverStation.getAlliance().isPresent) {
      if (FMSData.isBlue) {
        if (closestReefTagID in LEDConstants.BLUE_REEF_TAGS_BACK) {
          return LEDConstants.CandleState.REEF_TAG_BACK
        } else if (closestReefTagID in LEDConstants.BLUE_REEF_TAGS_FRONT) {
          return LEDConstants.CandleState.REEF_TAG_FRONT
        }
      } else {
        if (closestReefTagID in LEDConstants.RED_REEF_TAGS_BACK) {
          return LEDConstants.CandleState.REEF_TAG_BACK
        } else if (closestReefTagID in LEDConstants.RED_REEF_TAGS_FRONT) {
          return LEDConstants.CandleState.REEF_TAG_FRONT
        }
      }
    }
    return LEDConstants.CandleState.NO_REEF_TAG
  }

  fun periodic() {
    io.updateInputs(inputs)
    if (DriverStation.getAlliance().isEmpty) {
      var batteryVoltage = RobotController.getBatteryVoltage().volts

      state =
        if (batteryVoltage < 12.3.volts) {
          LEDConstants.CandleState.LOW_BATTERY_WARNING
        } else if (isTuningDebugging) {
          LEDConstants.CandleState.TUNING_MODE_WARNING
        } else {
          LEDConstants.CandleState.GOLD
        }
    } else if (DriverStation.isDisabled() && DriverStation.getAlliance().isPresent) {
      if (FMSData.isBlue) {
        state = LEDConstants.CandleState.BLUE
      } else {
        state = LEDConstants.CandleState.RED
      }
    } else if (isIntaking) {
      if (hasCoral) {
        state = LEDConstants.CandleState.INTAKE_SUCCESS
      } else {
        state = LEDConstants.CandleState.INTAKE_WAITING
      }
    } else {
      // if (hasCoral) {
      if (!isAutoAligning) {
        if (!seesTag) {
          state = LEDConstants.CandleState.NO_REEF_TAG
        } else {
          state = ledColorBasedOnTag()
        }
      } else {
        if (isAligned) {
          state = LEDConstants.CandleState.CAN_SCORE
        } else if (!seesTag) {
          state = LEDConstants.CandleState.NO_REEF_TAG
        } else {
          state = LEDConstants.CandleState.IS_ALIGNING
        }
      }
      // } else {
      // state = LEDConstants.CandleState.GOLD
      // }
    }

    Logger.processInputs("LED", inputs)
    CustomLogger.recordDebugOutput("LED/state", state.name)
  }
}
