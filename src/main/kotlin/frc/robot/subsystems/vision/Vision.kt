package frc.robot.subsystems.vision

import frc.robot.CURRENT_MODE
import frc.robot.lib.Mode
import java.util.Optional
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import org.photonvision.targeting.PhotonTrackedTarget
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.linalg.Matrix
import org.wpilib.math.linalg.VecBuilder
import org.wpilib.math.numbers.N1
import org.wpilib.math.numbers.N3
import org.wpilib.smartdashboard.Field2d

class Vision(val consumer: (Pose2d, Double, Matrix<N3, N1>) -> Unit) {
    private lateinit var curStdDevs: Matrix<N3, N1>
    private val photonCamera: PhotonCamera = PhotonCamera("Camera1")
    private val photonEstimator: PhotonPoseEstimator =
        PhotonPoseEstimator(tagLayout, robotToCam)
    private val visionSim: VisionSystemSim = VisionSystemSim("VisionSystemSim")
    private val camera = photonCamera

    init {
        if (CURRENT_MODE == Mode.SIM) {
            val visionSim = VisionSystemSim("main")
            visionSim.addAprilTags(tagLayout)
            var camraProp = SimCameraProperties()
            camraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90.0))
            camraProp.setCalibError(0.35, 0.10)
            camraProp.setFPS(15.0)
            camraProp.setAvgLatencyMs(50.0)
            camraProp.setLatencyStdDevMs(15.0)
            val camaraSim = PhotonCameraSim(camera, camraProp)
            visionSim.addCamera(camaraSim, transform3d)
            camaraSim.enableDrawWireframe(true)
        }
    }

    fun periodic() {
        var visionEst: Optional<EstimatedRobotPose>
        photonCamera.allUnreadResults.forEach { result ->
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result)
            if (visionEst.isEmpty) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result)
            }
            updateEstimationStdDevs(visionEst, result.getTargets())
            visionEst.ifPresent {
                consumer(
                    it.estimatedPose.toPose2d(),
                    it.timestampSeconds,
                    curStdDevs,
                )
                var estStdDevs = getEstimatationStdDevs()
            }
        }
    }

    private fun updateEstimationStdDevs(
        estimatedPose: Optional<EstimatedRobotPose>,
        targets: List<PhotonTrackedTarget>,
    ) {
        if (estimatedPose.isEmpty()) {
            curStdDevs = kSingleTagStdDevs
        } else {
            var estStdDevs = kSingleTagStdDevs
            var numTags: Int = 0
            var avgDist: Double = 0.0

            for (tgt in targets) {
                var tagPose =
                    photonEstimator.fieldTags.getTagPose(tgt.getFiducialId())
                if (tagPose.isEmpty()) continue
                numTags++
                avgDist +=
                    tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(
                            estimatedPose
                                .get()
                                .estimatedPose
                                .toPose2d()
                                .getTranslation()
                        )
            }
            if (numTags == 0) {
                curStdDevs = kSingleTagStdDevs
            } else {
                avgDist /= numTags
                if (numTags > 1) {
                    estStdDevs = kMultiTagStdDevs
                }
                if (numTags == 1 && avgDist > 4.0) {
                    estStdDevs =
                        VecBuilder.fill(
                            Double.MAX_VALUE,
                            Double.MAX_VALUE,
                            Double.MAX_VALUE,
                        )
                } else {
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30))
                    curStdDevs = estStdDevs
                }
            }
        }
    }

    fun getEstimatationStdDevs(): Matrix<N3, N1> {
        return curStdDevs
    }

    fun simulationPeriodic(pose2d: Pose2d, robotSimPose: Pose2d) {
        visionSim.update(robotSimPose)
    }

    fun resetSimPose(pose: Pose2d) {
        if (CURRENT_MODE == Mode.SIM) visionSim.resetRobotPose(pose)
    }

    fun getSimDebugField(): Field2d? {
        if (CURRENT_MODE == Mode.SIM) return null
        return visionSim.getDebugField()
    }

    @FunctionalInterface
    interface EstimateConsumer {
        fun accept(
            pose: Pose2d,
            timestamp: Double,
            estimationStdDevs: Matrix<N3, N1>,
        )
    }
}
