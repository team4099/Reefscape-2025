package com.team4099.robot2025.subsystems.vision.camera

import com.team4099.lib.hal.Clock
import com.team4099.lib.sim.utils.estimation.VisionEstimation
import com.team4099.robot2025.config.constants.VisionConstants
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.MultiTargetPNPResult
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PnpResult
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.seconds

class CameraIOPhotonvision(private val camera_details: VisionConstants.CameraDetailModel) : CameraIO {

    private val photonEstimator: PhotonPoseEstimator = PhotonPoseEstimator(
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera_details.cameraTransform.transform3d
    )
    private val camera: PhotonCamera = PhotonCamera(camera_details.cameraName)
    private var lastEstTimestamp: Time = 0.0.seconds
    private val distCoef = camera.distCoeffs.get()
    private var recentDetectionStream: MutableList<PhotonPipelineResult> = mutableListOf<PhotonPipelineResult>()


    fun processVisionStream(){
        val cameraReadStream = camera.allUnreadResults
        recentDetectionStream = cameraReadStream
            .filter { Clock.realTimestamp - it.timestampSeconds.seconds < 1.seconds }
            .map { it }.toMutableList() // Todo: cleanup this expression
        lastEstTimestamp = recentDetectionStream.last().timestampSeconds.seconds
    }

    override fun updateInputs(inputs: CameraIO.CameraInputs) {
        processVisionStream()
        inputs.cameraTargets = recentDetectionStream
        inputs.distCoeff = distCoef
        inputs.timestamp = lastEstTimestamp
    }

}