package com.team4099.robot2025.subsystems.drivetrain.drivev2

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog


interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        var driveConnected: Boolean = false
        var drivePositionRad: Double = 0.0
        var driveVelocityRadPerSec: Double = 0.0
        var driveAppliedVolts: Double = 0.0
        var driveCurrentAmps: Double = 0.0

        var turnConnected: Boolean = false
        var turnEncoderConnected: Boolean = false
        var turnAbsolutePosition: Rotation2d = Rotation2d()
        var turnPosition: Rotation2d = Rotation2d()
        var turnVelocityRadPerSec: Double = 0.0
        var turnAppliedVolts: Double = 0.0
        var turnCurrentAmps: Double = 0.0

        var odometryTimestamps: DoubleArray = doubleArrayOf()
        var odometryDrivePositionsRad: DoubleArray = doubleArrayOf()
        var odometryTurnPositions: Array<Rotation2d> = arrayOf()
    }

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ModuleIOInputs?) {}

    /** Run the drive motor at the specified open loop value.  */
    fun setDriveOpenLoop(output: Double) {}

    /** Run the turn motor at the specified open loop value.  */
    fun setTurnOpenLoop(output: Double) {}

    /** Run the drive motor at the specified velocity.  */
    fun setDriveVelocity(velocityRadPerSec: Double) {}

    /** Run the turn motor to the specified rotation.  */
    fun setTurnPosition(rotation: Rotation2d?) {}
}