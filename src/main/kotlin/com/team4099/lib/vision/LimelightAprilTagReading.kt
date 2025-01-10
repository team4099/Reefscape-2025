package com.team4099.lib.vision

import com.team4099.utils.LimelightHelpers
import com.team4099.utils.LimelightHelpers.LimelightTarget_Detector
import org.team4099.lib.units.base.Decimal
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees

class LimelightAprilTagReading(
    val tagID: Int,
    val tx: Angle,
    val ty: Angle,
    val ta: Decimal,
    val distToCamera: Length,
    val distToRobot: Length,
    val ambiguity: Double
){
    constructor(
        limelightReading: LimelightHelpers.RawFiducial
    ) : this(
        limelightReading.id,
        limelightReading.txnc.degrees,
        limelightReading.tync.degrees,
        limelightReading.ta.percent,
        limelightReading.distToCamera.meters,
        limelightReading.distToRobot.meters,
        limelightReading.ambiguity
    )

}