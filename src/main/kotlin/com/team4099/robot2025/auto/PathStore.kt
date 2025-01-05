package com.team4099.robot2023.auto

import com.pathplanner.lib.path.PathPlannerPath
import com.team4099.robot2023.config.constants.DrivetrainConstants
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController

object PathStore {
  val examplePath: PathPlannerPath = PathPlannerPath.fromPathFile("Example Path")

  val constraints =
    PathPlannerHolonomicDriveController.Companion.PathConstraints(
      DrivetrainConstants.MAX_AUTO_VEL,
      DrivetrainConstants.MAX_AUTO_ACCEL,
      DrivetrainConstants.STEERING_VEL_MAX,
      DrivetrainConstants.STEERING_ACCEL_MAX
    )
}
