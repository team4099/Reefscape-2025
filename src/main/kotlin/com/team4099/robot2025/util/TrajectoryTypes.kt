package com.team4099.robot2025.util

import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory as ChoreoTrajectory
import edu.wpi.first.math.trajectory.Trajectory as WPILibTrajectory

sealed interface TrajectoryTypes {
    class WPILib(val rawTrajectory: WPILibTrajectory) : TrajectoryTypes
    class Choreo(val rawTrajectory: ChoreoTrajectory<SwerveSample>) : TrajectoryTypes {
    }
}