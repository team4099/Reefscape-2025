package com.team4099.robot2023.config

import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.team4099.lib.joystick.XboxOneGamepad
import java.util.function.Consumer
import kotlin.math.absoluteValue

/**
 * Maps buttons on the driver and operator controllers to specific actions with meaningful variable
 * names.
 */
object ControlBoard {
  private val driver = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
  private val operator = XboxOneGamepad(Constants.Joysticks.SHOTGUN_PORT)
  private val technician = XboxOneGamepad(Constants.Joysticks.TECHNICIAN_PORT)

  val rumbleConsumer =
    Consumer<Boolean> {
      driver.setRumble(GenericHID.RumbleType.kBothRumble, if (it) 1.0 else 0.0)
    }

  val strafe: Double
    get() = -driver.leftXAxis

  val forward: Double
    get() = -driver.leftYAxis

  val turn: Double
    get() {
      return if (driver.rightXAxis.absoluteValue < 0.90) {
        driver.rightXAxis * DrivetrainConstants.TELEOP_TURNING_SPEED_PERCENT
      } else {
        driver.rightXAxis
      }
    }

  val slowMode: Boolean
    get() = driver.leftShoulderButton

  val resetGyro = Trigger { driver.startButton && driver.selectButton }

  // sim triggers
  val score = Trigger { driver.leftTriggerAxis > 0.5 }
  val intake = Trigger { driver.rightShoulderButton }
  val forceIdle = Trigger { driver.dPadDown || operator.startButton && operator.selectButton }

  // sim trigger
  // val score = Trigger {driver.bButton}
  // val intake = Trigger { driver.xButton}

  // real triggers
  // val score = Trigger {driver.leftTriggerAxis > 0.5}
  // val intake = Trigger { driver.rightShoulderButton}

  val targetAmp = Trigger { driver.aButton }
  val prepAmp = Trigger { operator.aButton }
  val prepLow = Trigger { operator.xButton }
  val prepHighProtected = Trigger { operator.bButton }
  val prepHigh = Trigger { operator.yButton }
  val passingShot = Trigger { operator.leftShoulderButton }
  val passingShotAlignment = Trigger { driver.yButton }
  val underStagePassingShot = Trigger { operator.rightShoulderButton }

  val extendClimb = Trigger { operator.dPadUp }
  val retractClimb = Trigger { operator.dPadDown }

  val ejectGamePiece = Trigger { driver.rightTriggerAxis > 0.5 }

  val testWrist = Trigger { driver.aButton }

  val characterizeWrist = Trigger { driver.rightShoulderButton }

  val climbAlignFar = Trigger { driver.dPadUp }
  val climbAlignLeft = Trigger { driver.dPadLeft }
  val climbAlignRight = Trigger { driver.dPadRight }

  val targetSpeaker = Trigger { driver.xButton } // TODO: switch back to climbAlignLeft
  val climbAutoAlign = Trigger { driver.bButton }

  val lockWheels = Trigger { driver.startButton }

  // week0 controls
}
