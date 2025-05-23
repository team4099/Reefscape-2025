package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.milli
import org.team4099.lib.units.perSecond

object Constants {
  object Universal {
    val gravity = -9.8.meters.perSecond.perSecond
    val SIM_MODE = Tuning.SimType.SIM
    const val REAL_FIELD = false

    const val CTRE_CONFIG_TIMEOUT = 0
    const val EPSILON = 1E-9

    val SLOW_STATUS_FRAME_TIME = 255.milli.seconds
    const val CANIVORE_NAME = "FalconVore"
    val LOG_FOLDER = "/media/sda1"

    val LOOP_PERIOD_TIME = 20.milli.seconds
    val POWER_DISTRIBUTION_HUB_ID = 1

    enum class GamePiece {
      CORAL,
      ALGAE,
      NONE
    }

    enum class CoralLevel {
      NONE,
      L1,
      L2,
      L3,
      L4
    }

    enum class AlgaeLevel {
      NONE,
      GROUND,
      L2,
      L3
    }
  }

  object AprilTagIds {
    const val BLUE_DOUBLE_SUBSTATION_ID = 4
  }

  object Tuning {
    const val TUNING_MODE = false
    const val DEBUGING_MODE = true
    const val SIMULATE_DRIFT = false
    const val DRIFT_CONSTANT = 0.001

    enum class SimType {
      SIM,
      REPLAY
    }
  }

  object Joysticks {
    const val DRIVER_PORT = 0
    const val SHOTGUN_PORT = 1
    const val TECHNICIAN_PORT = 2

    const val THROTTLE_DEADBAND = 0.05
    const val TURN_DEADBAND = 0.05
  }

  object Drivetrain {

    enum class DrivetrainType {
      PHOENIX_TALON,
      REV_NEO
    }

    val DRIVETRAIN_TYPE = DrivetrainType.PHOENIX_TALON

    const val FRONT_LEFT_DRIVE_ID = 11
    const val FRONT_LEFT_STEERING_ID = 21
    const val FRONT_LEFT_ANALOG_POTENTIOMETER = 1

    val FRONT_LEFT_MODULE_NAME = "Front Left Wheel"

    const val FRONT_RIGHT_DRIVE_ID = 12
    const val FRONT_RIGHT_STEERING_ID = 22
    const val FRONT_RIGHT_ANALOG_POTENTIOMETER = 2

    val FRONT_RIGHT_MODULE_NAME = "Front Right Wheel"

    const val BACK_LEFT_DRIVE_ID = 13
    const val BACK_LEFT_STEERING_ID = 23
    const val BACK_LEFT_ANALOG_POTENTIOMETER = 0

    val BACK_LEFT_MODULE_NAME = "Back Left Wheel"

    const val BACK_RIGHT_DRIVE_ID = 14
    const val BACK_RIGHT_STEERING_ID = 24
    const val BACK_RIGHT_ANALOG_POTENTIOMETER = 3

    val BACK_RIGHT_MODULE_NAME = "Back Right Wheel"
  }

  object Climber {
    const val CLIMBER_MOTOR_ID = 31
  }

  // leader is right motor and follower is left
  object Elevator {
    const val LEADER_MOTOR_ID = 42 // right
    const val FOLLOWER_MOTOR_ID = 41 // left
  }

  object Gyro {
    const val PIGEON_2_ID = 1
  }

  object Arm {
    const val ARM_MOTOR_ID = 51
    const val CANCODER_ID = 52
  }

  object Rollers {
    const val ROLLERS_MOTOR_ID = 61
    const val CANDI_ID = 62
  }

  object Ramp {
    const val RAMP_MOTOR_ID = 62
  }

  object Alert {
    val TABS = arrayOf("Pre-match", "In-match")
  }

  object LED {
    const val LED_CANDLE_ID = 1
  }
}
