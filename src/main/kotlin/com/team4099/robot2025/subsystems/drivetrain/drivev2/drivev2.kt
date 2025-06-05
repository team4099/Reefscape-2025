package com.team4099.robot2025.subsystems.drivetrain.drivev2

import com.ctre.phoenix6.CANBus
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.PathPlannerLogging
import com.team4099.robot2025.subsystems.drivetrain.gyro.GyroIO
import com.team4099.utils.threads.PhoenixOdometryThread
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.Constants
import frc.robot.Constants.Mode
import frc.robot.generated.TunerConstants
import frc.robot.util.LocalADStarAK
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import kotlin.math.hypot
import kotlin.math.max

class Drive(
    private val gyroIO: GyroIO,
    flModuleIO: ModuleIO?,
    frModuleIO: ModuleIO?,
    blModuleIO: ModuleIO?,
    brModuleIO: ModuleIO?
) : SubsystemBase() {
    private val gyroInputs: GyroIOInputsAutoLogged = GyroIOInputsAutoLogged()
    private val modules = arrayOfNulls<Module>(4) // FL, FR, BL, BR
    private val sysId: SysIdRoutine
    private val gyroDisconnectedAlert =
        Alert("Disconnected gyro, using kinematics as fallback.", Alert.AlertType.kError)

    private val kinematics = SwerveDriveKinematics(*moduleTranslations)
    private var rawGyroRotation = Rotation2d()
    private val lastModulePositions =  // For delta tracking
        arrayOf<SwerveModulePosition?>(
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition()
        )
    private val poseEstimator = SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d())

    init {
        modules[0] = Module(flModuleIO, 0, TunerConstants.FrontLeft)
        modules[1] = Module(frModuleIO, 1, TunerConstants.FrontRight)
        modules[2] = Module(blModuleIO, 2, TunerConstants.BackLeft)
        modules[3] = Module(brModuleIO, 3, TunerConstants.BackRight)

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit)

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start()

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            { this.pose },
            { pose: Pose2d? ->
                this.pose =
                    pose
            },
            { this.chassisSpeeds },
            { speeds: ChassisSpeeds? ->
                this.runVelocity(
                    speeds
                )
            },
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0), PIDConstants(5.0, 0.0, 0.0)
            ),
            PP_CONFIG,
            { DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red },
            this
        )
        Pathfinding.setPathfinder(LocalADStarAK())
        PathPlannerLogging.setLogActivePathCallback { activePath: List<Pose2d> ->
            Logger.recordOutput(
                "Odometry/Trajectory", *activePath.toTypedArray<Pose2d>()
            )
        }
        PathPlannerLogging.setLogTargetPoseCallback { targetPose: Pose2d ->
            Logger.recordOutput(
                "Odometry/TrajectorySetpoint",
                targetPose
            )
        }

        // Configure SysId
        sysId =
            SysIdRoutine(
                SysIdRoutine.Config(
                    null,
                    null,
                    null
                ) { state: SysIdRoutineLog.State ->
                    Logger.recordOutput(
                        "Drive/SysIdState",
                        state.toString()
                    )
                },
                Mechanism(
                    { voltage: Voltage -> runCharacterization(voltage.`in`(Units.Volts)) }, null, this
                )
            )
    }

    override fun periodic() {
        odometryLock.lock() // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("Drive/Gyro", gyroInputs)
        for (module in modules) {
            module.periodic()
        }
        odometryLock.unlock()

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (module in modules) {
                module.stop()
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", *arrayOf<SwerveModuleState>())
            Logger.recordOutput("SwerveStates/SetpointsOptimized", *arrayOf<SwerveModuleState>())
        }

        // Update odometry
        val sampleTimestamps: DoubleArray =
            modules[0].getOdometryTimestamps() // All signals are sampled together
        val sampleCount = sampleTimestamps.size
        for (i in 0 until sampleCount) {
            // Read wheel positions and deltas from each module
            val modulePositions = arrayOfNulls<SwerveModulePosition>(4)
            val moduleDeltas = arrayOfNulls<SwerveModulePosition>(4)
            for (moduleIndex in 0..3) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions().get(i)
                moduleDeltas[moduleIndex] =
                    SwerveModulePosition(
                        modulePositions[moduleIndex]!!.distanceMeters
                                - lastModulePositions[moduleIndex]!!.distanceMeters,
                        modulePositions[moduleIndex]!!.angle
                    )
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex]
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions.get(i)
            } else {
                // Use the angle delta from the kinematics and module deltas
                val twist = kinematics.toTwist2d(*moduleDeltas)
                rawGyroRotation = rawGyroRotation.plus(Rotation2d(twist.dtheta))
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions)
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode !== Mode.SIM)
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    fun runVelocity(speeds: ChassisSpeeds?) {
        // Calculate module setpoints
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts)

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds)

        // Send setpoints to modules
        for (i in 0..3) {
            modules[i].runSetpoint(setpointStates[i])
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", *setpointStates)
    }

    /** Runs the drive in a straight line with the specified drive output.  */
    fun runCharacterization(output: Double) {
        for (i in 0..3) {
            modules[i].runCharacterization(output)
        }
    }

    /** Stops the drive.  */
    fun stop() {
        runVelocity(ChassisSpeeds())
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    fun stopWithX() {
        val headings = arrayOfNulls<Rotation2d>(4)
        for (i in 0..3) {
            headings[i] = moduleTranslations[i].angle
        }
        kinematics.resetHeadings(*headings)
        stop()
    }

    /** Returns a command to run a quasistatic test in the specified direction.  */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
        return run { runCharacterization(0.0) }
            .withTimeout(1.0)
            .andThen(sysId.quasistatic(direction))
    }

    /** Returns a command to run a dynamic test in the specified direction.  */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command {
        return run { runCharacterization(0.0) }.withTimeout(1.0).andThen(sysId.dynamic(direction))
    }

    @get:AutoLogOutput(key = "SwerveStates/Measured")
    private val moduleStates: Array<SwerveModuleState?>
        /** Returns the module states (turn angles and drive velocities) for all of the modules.  */
        get() {
            val states = arrayOfNulls<SwerveModuleState>(4)
            for (i in 0..3) {
                states[i] = modules[i].getState()
            }
            return states
        }

    private val modulePositions: Array<SwerveModulePosition?>
        /** Returns the module positions (turn angles and drive positions) for all of the modules.  */
        get() {
            val states = arrayOfNulls<SwerveModulePosition>(4)
            for (i in 0..3) {
                states[i] = modules[i].getPosition()
            }
            return states
        }

    @get:AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private val chassisSpeeds: ChassisSpeeds
        /** Returns the measured chassis speeds of the robot.  */
        get() = kinematics.toChassisSpeeds(*moduleStates)

    val wheelRadiusCharacterizationPositions: DoubleArray
        /** Returns the position of each module in radians.  */
        get() {
            val values = DoubleArray(4)
            for (i in 0..3) {
                values[i] = modules[i].getWheelRadiusCharacterizationPosition()
            }
            return values
        }

    val fFCharacterizationVelocity: Double
        /** Returns the average velocity of the modules in rotations/sec (Phoenix native units).  */
        get() {
            var output = 0.0
            for (i in 0..3) {
                output += modules[i].getFFCharacterizationVelocity() / 4.0
            }
            return output
        }

    @get:AutoLogOutput(key = "Odometry/Robot")
    var pose: Pose2d?
        /** Returns the current odometry pose.  */
        get() = poseEstimator.estimatedPosition
        /** Resets the current odometry pose.  */
        set(pose) {
            poseEstimator.resetPosition(rawGyroRotation, modulePositions, pose)
        }

    val rotation: Rotation2d
        /** Returns the current odometry rotation.  */
        get() = pose!!.rotation

    /** Adds a new timestamped vision measurement.  */
    fun addVisionMeasurement(
        visionRobotPoseMeters: Pose2d?,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3?, N1?>?
    ) {
        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs
        )
    }

    val maxLinearSpeedMetersPerSec: Double
        /** Returns the maximum linear speed in meters per sec.  */
        get() = TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond)

    val maxAngularSpeedRadPerSec: Double
        /** Returns the maximum angular speed in radians per sec.  */
        get() = maxLinearSpeedMetersPerSec / DRIVE_BASE_RADIUS

    companion object {
        // TunerConstants doesn't include these constants, so they are declared locally
        val ODOMETRY_FREQUENCY: Double =
            if (CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD) 250.0 else 100.0
        val DRIVE_BASE_RADIUS: Double = max(
            max(
                hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)
            ),
            max(
                hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
            )
        )

        // PathPlanner config constants
        private const val ROBOT_MASS_KG = 74.088
        private const val ROBOT_MOI = 6.883
        private const val WHEEL_COF = 1.2
        private val PP_CONFIG = RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            ModuleConfig(
                TunerConstants.FrontLeft.WheelRadius,
                TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond),
                WHEEL_COF,
                DCMotor.getKrakenX60Foc(1)
                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                TunerConstants.FrontLeft.SlipCurrent,
                1
            ),
            *moduleTranslations
        )

        val odometryLock: Lock = ReentrantLock()
        val moduleTranslations: Array<Translation2d>
            /** Returns an array of module translations.  */
            get() = arrayOf<Translation2d>(
                Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
                Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
            )
    }
}