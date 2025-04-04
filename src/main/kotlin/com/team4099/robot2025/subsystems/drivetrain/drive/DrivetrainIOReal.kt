package com.team4099.robot2025.subsystems.drivetrain.drive

import com.ctre.phoenix6.hardware.TalonFX
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.Constants.Universal.CANIVORE_NAME
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.swervemodule.SwerveModule
import com.team4099.robot2025.subsystems.drivetrain.swervemodule.SwerveModuleIOTalon
import edu.wpi.first.wpilibj.AnalogInput

object DrivetrainIOReal : DrivetrainIO {
  override fun getSwerveModules(): List<SwerveModule> {
    return listOf(
      SwerveModule(
        SwerveModuleIOTalon(
          TalonFX(Constants.Drivetrain.FRONT_LEFT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.FRONT_LEFT_DRIVE_ID, CANIVORE_NAME),
          AnalogInput(Constants.Drivetrain.FRONT_LEFT_ANALOG_POTENTIOMETER),
          DrivetrainConstants.FRONT_LEFT_MODULE_ZERO,
          DrivetrainConstants.MK4N_STEERING_SENSOR_GEAR_RATIO,
          Constants.Drivetrain.FRONT_LEFT_MODULE_NAME
        )
      ),
      SwerveModule(
        SwerveModuleIOTalon(
          TalonFX(Constants.Drivetrain.FRONT_RIGHT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.FRONT_RIGHT_DRIVE_ID, CANIVORE_NAME),
          AnalogInput(Constants.Drivetrain.FRONT_RIGHT_ANALOG_POTENTIOMETER),
          DrivetrainConstants.FRONT_RIGHT_MODULE_ZERO,
          DrivetrainConstants.MK4N_STEERING_SENSOR_GEAR_RATIO,
          Constants.Drivetrain.FRONT_RIGHT_MODULE_NAME
        )
      ),
      SwerveModule(
        SwerveModuleIOTalon(
          TalonFX(Constants.Drivetrain.BACK_LEFT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.BACK_LEFT_DRIVE_ID, CANIVORE_NAME),
          AnalogInput(Constants.Drivetrain.BACK_LEFT_ANALOG_POTENTIOMETER),
          DrivetrainConstants.BACK_LEFT_MODULE_ZERO,
          DrivetrainConstants.MK4I_STEERING_SENSOR_GEAR_RATIO,
          Constants.Drivetrain.BACK_LEFT_MODULE_NAME
        )
      ),
      SwerveModule(
        SwerveModuleIOTalon(
          TalonFX(Constants.Drivetrain.BACK_RIGHT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.BACK_RIGHT_DRIVE_ID, CANIVORE_NAME),
          AnalogInput(Constants.Drivetrain.BACK_RIGHT_ANALOG_POTENTIOMETER),
          DrivetrainConstants.BACK_RIGHT_MODULE_ZERO,
          DrivetrainConstants.MK4I_STEERING_SENSOR_GEAR_RATIO,
          Constants.Drivetrain.BACK_RIGHT_MODULE_NAME
        )
      )
    )
  }
}
