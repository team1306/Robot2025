// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import badgerlog.entry.handlers.Key;
import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import badgerlog.Dashboard;
import badgerlog.entry.Entry;
import badgerlog.entry.EntryType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.commands.autos.FieldLocation;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.Utilities;
import lombok.Getter;
import lombok.SneakyThrows;
import org.json.simple.parser.ParseException;
import swervelib.*;
import swervelib.imu.SwerveIMU;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.LIMELIGHT_3_NAME;
import static frc.robot.Constants.LIMELIGHT_4_NAME;

public class SwerveSubsystem extends SubsystemBase {
    @Getter
    private final SwerveDrive swerveDrive;

    private final PIDController autoXController = new PIDController(7, 0, 0.2);
    private final PIDController autoYController = new PIDController(7, 0, 0.2);

    @Entry(EntryType.Subscriber)
    private static double WRIST_POSE_SHIFT = 1;

    private final PIDController autoHeadingController = new PIDController(3.5, 0, 0.1);

    @Entry(EntryType.Sendable)
    @Key("Auto/Translation Controller")
    private static ProfiledPIDController translationController = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(2, 1));

    @Entry(EntryType.Sendable)
    @Key("Auto/Heading Controller")
    private static ProfiledPIDController headingController = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(50, 25));

    private final SwerveInputStream driveToReefPose;
    private final SwerveInputStream driveToCoralStationPose;
    private final IntSupplier wristMultSupplier;
    private final IntSupplier levelSupplier;

    public SwerveSubsystem(IntSupplier wristMultSupplier, IntSupplier levelSupplier) {
        this.wristMultSupplier = wristMultSupplier;
        this.levelSupplier = levelSupplier;

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            File directory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED); // 5 meters per second, temp value      // Alternative method if you don't want to supply the conversion factor via JSON files.
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(false, false, 0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
        swerveDrive.setAutoCenteringModules(false);

        autoHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        translationController.setTolerance(0.001);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(0.001);
        driveToReefPose = SwerveInputStream.of(swerveDrive, () -> 0, () -> 0)
                .driveToPose(this::getNearestReefLocation, translationController, headingController)
                .driveToPoseEnabled(true);
        driveToCoralStationPose = SwerveInputStream.of(swerveDrive, () -> 0, () -> 0)
                .driveToPose(this::getNearestCoralStationLocation, translationController, headingController)
                .driveToPoseEnabled(true);

        /*
        0 - use external imu,
        1 - use external imu, seed internal imu,
        2 - use internal,
        3 - use internal with MT1 assisted convergence,
        4 - use internal IMU with external IMU assisted convergence
        */
        LimelightHelpers.SetIMUMode(LIMELIGHT_4_NAME, 0);
//        replaceYAGSLIMU();
        setupPathPlanner();

        Dashboard.putValue("Auto/Reef Align Startup", false);
        Dashboard.putValue("Auto/Station Align Startup", false);
    }

    @SneakyThrows({NoSuchFieldException.class, IllegalAccessException.class})
    public void replaceYAGSLIMU() {
        LimelightIMU imu = new LimelightIMU();
        swerveDrive.swerveDriveConfiguration.imu = imu;
        swerveDrive.imuReadingCache.updateSupplier(imu::getRotation3d);
        Field field = swerveDrive.getClass().getDeclaredField("imu");
        field.setAccessible(true);
        field.set(swerveDrive, imu);
        imu.setOffset(imu.getRawRotation3d());
    }

    private Pose2d reefLastCachedLocation = null;
    private boolean reefEnabled = false;

    @Entry(EntryType.Sendable)
    private static Field2d shiftedPose = new Field2d();

    private Pose2d getNearestReefLocation() {
        if (reefEnabled && reefLastCachedLocation != null) return reefLastCachedLocation;
        reefLastCachedLocation = getReefLocationOdometryMethod();
        shiftedPose.setRobotPose(reefLastCachedLocation);
        return reefLastCachedLocation;
    }

    private Pose2d shiftPoseRelativeToIntake(Pose2d fieldRelativePose) {
        final Distance shift = Inches.of(WRIST_POSE_SHIFT)
                .times(-wristMultSupplier.getAsInt())
                .times(FieldLocation.reefLocations.get(fieldRelativePose) ? -1 : 1);
        final Distance forwardShift = Inches.of(switch (levelSupplier.getAsInt()) {
            case 3 -> 2;
            case 4 -> -1;
            default -> 0;
        });
        Pose2d transformedPose = fieldRelativePose.transformBy(new Transform2d(new Translation2d(forwardShift
                .in(Meter), shift.in(Meter)), fieldRelativePose.getRotation()));
        return new Pose2d(transformedPose.getTranslation(), fieldRelativePose.getRotation());
    }

    private Pose2d getReefLocationOdometryMethod() {
        Pose2d position = getPose().nearest(FieldLocation.reefLocations.keySet().stream().toList());
        position = shiftPoseRelativeToIntake(position);
        return position;
    }

    private Command reefAutoAlign = null;

    public Command getReefAutoAlignCommand() {
        if (reefAutoAlign == null) reefAutoAlign = driveFieldOriented(driveToReefPose);
        return reefAutoAlign;
    }

    private Pose2d coralStationLastCachedLocation = null;
    private boolean coralStationEnabled = false;
    @Entry(EntryType.Sendable)
    private static Field2d coralStationField = new Field2d();

    private Pose2d getNearestCoralStationLocation() {
        if (coralStationEnabled && coralStationLastCachedLocation != null) return coralStationLastCachedLocation;
        coralStationLastCachedLocation = getPose().nearest(FieldLocation.coralStationLocations);
        coralStationField.setRobotPose(coralStationLastCachedLocation);
        return coralStationLastCachedLocation;
    }

    private Command coralStationAutoAlign = null;

    public Command getCoralStationAutoAlign() {
        if (coralStationAutoAlign == null) coralStationAutoAlign = driveFieldOriented(driveToCoralStationPose);
        return coralStationAutoAlign;
    }

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + autoXController.calculate(pose.getX(), sample.x), sample.vy + autoYController.calculate(pose
                        .getY(), sample.y), sample.omega + autoHeadingController.calculate(pose.getRotation()
                                .getRadians(), sample.heading)
        );

        // Apply the generated speeds
        driveFieldOriented(speeds);
    }

    @Override
    public void periodic() {
        reefEnabled = getReefAutoAlignCommand().isScheduled();
        coralStationEnabled = getCoralStationAutoAlign().isScheduled();

        if (validateVelocity(driveToReefPose.get())) Dashboard.putValue("Auto/Reef Align Startup", true);
        if (validateVelocity(driveToCoralStationPose.get())) Dashboard.putValue("Auto/Station Align Startup", true);

        Dashboard.putValue("Auto/TranslationError", translationController.getPositionError());
        Dashboard.putValue("Auto/HeadingError", headingController.getPositionError());

        addVisionMeasurement(LIMELIGHT_4_NAME);
        addVisionMeasurement(LIMELIGHT_3_NAME);
    }

    public void addVisionMeasurement(String limelightName) {
        LimelightHelpers.SetRobotOrientation(limelightName, swerveDrive.getPose()
                .getRotation()
                .getDegrees(), 0, 0, 0, 0, 0);
        PoseEstimate poseEstimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (poseEstimateMT2 == null) return;

        Pose2d pose = new Pose2d(poseEstimateMT2.pose.getTranslation(), swerveDrive.getPose().getRotation());
        if (poseEstimateMT2.tagCount >= 1) swerveDrive.addVisionMeasurement(pose, poseEstimateMT2.timestampSeconds);

        //If the pose can be determined to be very likely accurate, use MT1 for gyro
        if (LimelightHelpers.getTA(limelightName) < 0.6) return;

        PoseEstimate poseEstimateMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (poseEstimateMT1 == null) return;

        if (poseEstimateMT1.tagCount >= 1) swerveDrive
                .addVisionMeasurement(poseEstimateMT1.pose, poseEstimateMT1.timestampSeconds);
    }

    public Command setModuleAngleSetpoint(Rotation2d angle) {
        return new InstantCommand(() -> {
            for (SwerveModule swerveModule : swerveDrive.getModules()) {
                swerveModule.getAngleMotor().setReference(angle.getDegrees(), 0);
            }
        });
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    @SneakyThrows({ParseException.class, IOException.class})
    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;

        config = RobotConfig.fromGUISettings();

        AutoBuilder.configure(
                this::getPose, this::resetOdometry, this::getRobotVelocity, (speedsRobotRelative, moduleFeedForwards) -> {
                    swerveDrive.setChassisSpeeds(speedsRobotRelative);
                }, new PPHolonomicDriveController(
                        // Translation PID constants
                        new PIDConstants(autoXController.getP(), autoXController.getI(), autoXController.getD()),
                        // Rotation PID constants
                        new PIDConstants(autoHeadingController.getP(), autoHeadingController
                                .getI(), autoHeadingController.getD())
                ), config, Utilities::isRedAlliance, this);

        //Preload PathPlanner Path finding
        // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(), this, swerveDrive, 12, true), 3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(), this, swerveDrive), 3.0, 5.0, 3.0);
    }

    /**
     * Returns a Command that centers the modules of the SwerveDrive subsystem.
     *
     * @return a Command that centers the modules of the SwerveDrive subsystem
     */
    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
    }

    /**
     * Returns a Command that drives the swerve drive to a specific distance at a given speed.
     *
     * @param distanceInMeters the distance to drive in meters
     * @param speedInMetersPerSecond the speed at which to drive in meters per second
     *
     * @return a Command that drives the swerve drive to a specific distance at a given speed
     */
    public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
        return new InstantCommand(() -> resetOdometry(new Pose2d()))
                .andThen(run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
                        .until(() -> swerveDrive.getPose()
                                .getTranslation()
                                .getDistance(new Translation2d()) > distanceInMeters));
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(velocity, getHeading()));
    }

    private double swerveSpeed = 1;

    public Command changeSwerveSpeed(double speed) {
        return new InstantCommand(() -> this.swerveSpeed = speed);
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> drive(ChassisSpeeds.fromFieldRelativeSpeeds(velocity.get(), getHeading())));
    }

    public boolean validateVelocity(ChassisSpeeds velocity) {
        return !velocity.equals(new ChassisSpeeds());
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity.times(swerveSpeed));
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public Command drive(Supplier<ChassisSpeeds> velocity) {
        return run(() -> drive(velocity.get()));
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing forward
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    public void zeroGyroWithAlliance() {
        if (Utilities.isRedAlliance()) {
            zeroGyro();
            //Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else
            zeroGyro();

    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    public static class LimelightIMU extends SwerveIMU {
        private final MutAngularVelocity angularVelocity = new MutAngularVelocity(0, 0, DegreesPerSecond);

        private Rotation3d offset = Rotation3d.kZero;
        private boolean inverted = false;

        private double lastYaw = 0;

        @Override
        public void setOffset(Rotation3d offset) {
            this.offset = offset;
        }

        @Override
        public void setInverted(boolean invertIMU) {
            inverted = invertIMU;
        }

        @Override
        public Rotation3d getRawRotation3d() {
            LimelightHelpers.IMUData data = LimelightHelpers.getIMUData(LIMELIGHT_4_NAME);
            Rotation3d rawRotation = new Rotation3d(Math.toRadians(data.Roll), Math.toRadians(data.Pitch), Math
                    .toRadians(data.Yaw)).times(inverted ? -1 : 1);
            Dashboard.putValue("LimelightIMU/Rotation Raw", rawRotation);
            return rawRotation;
        }

        @Override
        public Rotation3d getRotation3d() {
            Rotation3d rotation = getRawRotation3d().minus(offset);
            Dashboard.putValue("LimelightIMU/Rotation Adjusted", rotation);
            return rotation;
        }

        @Override
        public Optional<Translation3d> getAccel() {
            LimelightHelpers.IMUData data = LimelightHelpers.getIMUData(LIMELIGHT_4_NAME);

            Translation3d accel = new Translation3d(data.accelX, data.accelY, data.accelZ);
            Dashboard.putValue("LimelightIMU/Acceleration", accel);
            return Optional.of(accel);
        }

        @Override
        public MutAngularVelocity getYawAngularVelocity() {
            double currentYaw = LimelightHelpers.getIMUData(LIMELIGHT_4_NAME).Yaw;
            double yawVelocity = (currentYaw - lastYaw) / 0.02;
            lastYaw = currentYaw;
            Dashboard.putValue("LimelightIMU/Yaw Velocity", yawVelocity);
            return angularVelocity.mut_setMagnitude(yawVelocity);
        }

        @Override
        public Object getIMU() {
            //Limelight is a network tables publisher so it has no object
            return null;
        }

        @Override
        public void close() {
            //No resources to release
        }

        @Override
        public void factoryDefault() {
            //Limelight has onboard computer so no factory default
        }

        @Override
        public void clearStickyFaults() {
            //Not possible for sticky faults on limelight
        }
    }
}
