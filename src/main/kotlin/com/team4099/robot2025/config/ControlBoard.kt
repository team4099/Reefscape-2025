package com.team4099.robot2025.config

import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
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
  val forceIdle = Trigger { driver.dPadDown }

  //Tuning Binds
  val testElevatorBind = Trigger { driver.aButton }
  val testElevatorDownBind = Trigger { driver.bButton }
  val testClimberBind = Trigger { driver.bButton }
  val testRollersBind = Trigger { driver.xButton }
  val testArmBind = Trigger { driver.yButton }

  //Single Driver Binds
  val prepL2 = Trigger { driver.aButton }
  val prepL3 = Trigger { driver.bButton }
  val prepL4 = Trigger { driver.yButton }

  val score = Trigger { driver.rightTriggerAxis > 0.5 }
  val intakeCoral = Trigger { driver.leftTriggerAxis > 0.5 }

}
