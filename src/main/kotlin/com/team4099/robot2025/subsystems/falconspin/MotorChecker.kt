package com.team4099.robot2025.subsystems.falconspin


import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.DriverStation
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.inVolts

object MotorChecker {

    val subsystemHardware = HashMap<String, HashMap<String, MutableList<MotorCollection>>>()

    private val baseStageCurrentLimitTriggered = ArrayList<String>()
    private val firstStageCurrentLimitTriggered = ArrayList<String>()
    private val shutdownTriggered = ArrayList<String>()

    fun add(subsystemName: String, subCategory: String, subsystemMotorCollections: MotorCollection) {
        if (subsystemHardware[subsystemName] == null) {
            subsystemHardware[subsystemName] = HashMap()
        }
        if (subsystemHardware[subsystemName]?.get(subCategory) == null) {
            subsystemHardware[subsystemName]?.set(subCategory, mutableListOf())
        }

        subsystemHardware[subsystemName]!![subCategory]!!.add(subsystemMotorCollections)
    }

    fun periodic() {
        CustomLogger.recordDebugOutput(
            "MotorChecker/subsystemNames", subsystemHardware.keys.toTypedArray()
        )

        for (subsystemName in subsystemHardware.keys) {

            val motorNames = mutableListOf<String>()
            for (subCategory in subsystemHardware[subsystemName]!!) {
                for (motorCollection in subCategory.value) {

                    motorNames.addAll(motorCollection.motorCollection.map { it.name })

                    // base current limit
                    if (motorCollection.maxMotorTemperature < motorCollection.firstStageTemperatureLimit &&
                        motorCollection.currentLimitStage != CURRENT_STAGE_LIMIT.BASE
                    ) {
                        baseStageCurrentLimitTriggered.add(subCategory.key)
                        motorCollection.setCurrentLimit(motorCollection.baseCurrentLimit)
                    }

                    // first stage current limit
                    if (motorCollection.maxMotorTemperature in
                        motorCollection.firstStageTemperatureLimit..motorCollection.motorShutDownThreshold &&
                        motorCollection.currentLimitStage != CURRENT_STAGE_LIMIT.FIRST
                    ) {
                        firstStageCurrentLimitTriggered.add(subCategory.key)
                        motorCollection.setCurrentLimit(motorCollection.firstStageCurrentLimit)
                    }

                    for (motor in motorCollection.motorCollection) {
                        // complete motor shutdown but we don't want to shut down all motors at once
                        if (!DriverStation.isFMSAttached()) {
                            if (motor.temperature > motor.motorShutDownThreshold) {
                                shutdownTriggered.add(subCategory.key)
                                motor.shutdown()
                            }
                        }

                        //            logMotor(subsystemName, motor)
                    }
                }
            }
            CustomLogger.recordDebugOutput(
                "MotorChecker/$subsystemName/motorNames", motorNames.toTypedArray()
            )

            Logger.recordOutput(
                "MotorChecker/baseStageTriggered", baseStageCurrentLimitTriggered.toTypedArray()
            )
            Logger.recordOutput(
                "MotorChecker/firstStageTriggered", firstStageCurrentLimitTriggered.toTypedArray()
            )
            Logger.recordOutput("MotorChecker/shutdownTriggered", shutdownTriggered.toTypedArray())
            baseStageCurrentLimitTriggered.clear()
            firstStageCurrentLimitTriggered.clear()
            shutdownTriggered.clear()
        }
    }
}

// not clean but whatever
fun logMotor(subsystemName: String, motor: Motor<MotorType>) {
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/AppliedVoltageVolts", motor.appliedVoltage.inVolts
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/BusVoltageVolts", motor.busVoltage.inVolts
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/TemperatureCelsius", motor.temperature.inCelsius
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/StatorCurrentAmps", motor.statorCurrent.inAmperes
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/SupplyCurrentAmps", motor.supplyCurrent.inAmperes
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/CurrentLimitStage", motor.currentLimitStage.name
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/BaseCurrentLimitAmps",
        motor.baseCurrentLimit.inAmperes
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/FirstStageTemperatureLimitCelsius",
        motor.firstStageTemperatureLimit.inCelsius
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/FirstStageCurrentLimitAmps",
        motor.firstStageCurrentLimit.inAmperes
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/MotorShutDownThresholdCelsius",
        motor.motorShutDownThreshold.inCelsius
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/CurrentLimitInUseAmps",
        motor.currentLimitInUse.inAmperes
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/MotorID", motor.id.toLong()
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/Errors", motor.errors.toTypedArray()
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/Warnings", motor.warnings.toTypedArray()
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/Info", motor.info.toTypedArray()
    )
    CustomLogger.recordDebugOutput(
        "MotorChecker/$subsystemName/${motor.name}/StickyFaults", motor.stickyFaults.toTypedArray()
    )
}