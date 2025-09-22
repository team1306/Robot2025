

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.Constants.cameraOffset;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import badgerlog.Dashboard;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;

    public final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    private Pose3d bestTarget = new Pose3d();

    private Pose2d robotposesim = new Pose2d();

    public Vision(String cameraName) {
        camera = new PhotonCamera(cameraName);

        photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraOffset);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagFieldLayout);
            //big circle
            TargetModel targetModel = new TargetModel(1);
            Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
            VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

            // Add this vision target to the vision system simulation to make it visible
            visionSim.addVisionTargets(visionTarget);
            SimCameraProperties cameraProp = new SimCameraProperties();

            cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(60);
            cameraProp.setAvgLatencyMs(10);
            cameraProp.setLatencyStdDevMs(15);

            cameraSim = new PhotonCameraSim(camera, cameraProp);

            visionSim.addCamera(cameraSim, cameraOffset);

            cameraSim.enableDrawWireframe(true);
        }

    }

    public Pose3d getObjectPose() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Pose3d pose = new Pose3d(target.getBestCameraToTarget().toMatrix()); // pose 3d

            return pose;
        } else {
            return new Pose3d();
        }
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);


            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est -> getSimDebugField()
                                .getObject("VisionEstimation")
                                .setPose(est.estimatedPose.toPose2d()), () -> {
                                    getSimDebugField().getObject("VisionEstimation").setPoses();
                                });
            }

            visionEst.ifPresent(
                    est -> {
                        Pose3d oink = est.estimatedPose;
                        Dashboard.putValue("Copoius amounts of uranium", oink);
                    });
        }
    }

    @Override
    public void simulationPeriodic() {

        visionSim.update(robotposesim);
        bestTarget = getObjectPose();

    }


    public void updateRobotPose(Pose2d pose) {
        robotposesim = pose;
        photonEstimator.getReferencePose();
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }


}
