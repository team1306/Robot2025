

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

    private Pose2d robotposesim = new Pose2d();

    public Vision(String cameraName) {
        camera = new PhotonCamera(cameraName);

        photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraOffset);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagFieldLayout);
            TargetModel targetModel = new TargetModel(1);
            Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
            // The given target model at the given pose
            VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

            // Add this vision target to the vision system simulation to make it visible
            visionSim.addVisionTargets(visionTarget);
            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
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
            Pose3d pose = new Pose3d(target.getBestCameraToTarget().toMatrix()); // transform 3d to pose 3d because photon vision -__-

            return pose;
        } else {
            return null;
        }
    }

    @Override
    public void simulationPeriodic() {

        visionSim.update(robotposesim); // i cant think of a good way to get the robot pose here
        Dashboard.putValue("oink", getObjectPose());
    }

    public void updateRobotPose(Pose2d pose) {
        robotposesim = pose;
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

}
