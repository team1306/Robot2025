// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LIMELIGHT_NAME;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.autos.FieldLocation;
import frc.robot.util.dashboardv3.Dashboard;
import frc.robot.util.dashboardv3.entry.Entry;
import frc.robot.util.dashboardv3.entry.EntryType;
import lombok.Setter;
import lombok.SneakyThrows;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.Utilities;
import lombok.Getter;
import swervelib.*;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    @Getter
    private final SwerveDrive swerveDrive;

    private final PIDController autoXController = new PIDController(7, 0, 0.2);
    private final PIDController autoYController = new PIDController(7, 0, 0.2);

    private final PIDController autoHeadingController = new PIDController(3.5, 0, 0.1);
    
    @Entry(key = "Auto/Translation Controller", type = EntryType.Sendable)
    private static ProfiledPIDController translationController =
            new ProfiledPIDController(4, 0.01, 0, new TrapezoidProfile.Constraints(2, 1));

    @Entry(key = "Auto/Heading Controller", type = EntryType.Sendable)
    private static ProfiledPIDController headingController =
            new ProfiledPIDController(4, 0.01, 0, new TrapezoidProfile.Constraints(50, 25));

    private final SwerveInputStream driveToPose;

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            File directory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED); // 5 meters per second, temp value      // Alternative method if you don't want to supply the conversion factor via JSON files.
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(true); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(false, false, 0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
        swerveDrive.setAutoCenteringModules(false);

        autoHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        translationController.setTolerance(0.001);
        headingController.setTolerance(0.001);
        driveToPose = SwerveInputStream.of(swerveDrive, () -> 0, () -> 0)
                .driveToPose(this::getNearestFieldLocation, translationController, headingController)
                .driveToPoseEnabled(true);
        
        setupPathPlanner();
    }

    private Pose2d lastCachedLocation = null;
    private boolean isEnabled = false;

    private Pose2d getNearestFieldLocation(){
        if(isEnabled && lastCachedLocation != null) return lastCachedLocation;
        lastCachedLocation = getPose().nearest(FieldLocation.reefLocations);
        return lastCachedLocation;
    }
    
    public Command getAutoAlignCommand(){
        return driveFieldOriented(driveToPose);
    }

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + autoXController.calculate(pose.getX(), sample.x),
                sample.vy + autoYController.calculate(pose.getY(), sample.y),
                sample.omega + autoHeadingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );
        
        // Apply the generated speeds
        driveFieldOriented(speeds);
    }

    @Override
    public void periodic() {
        isEnabled = getAutoAlignCommand().isScheduled();
        Dashboard.putValue("Auto/TranslationError", translationController.getPositionError());
        Dashboard.putValue("Auto/HeadingError", headingController.getPositionError());

        if(velocityDebounce.advanceIfElapsed(0.01)) accelerationLimiter.reset(0);
        addVisionMeasurement();
    }

    public void addVisionMeasurement(){
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME, swerveDrive.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        PoseEstimate poseEstimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        if(poseEstimateMT2 == null) return;

        Pose2d pose = new Pose2d(poseEstimateMT2.pose.getTranslation(), swerveDrive.getPose().getRotation());
        if(poseEstimateMT2.tagCount >= 1) swerveDrive.addVisionMeasurement(pose, poseEstimateMT2.timestampSeconds); 
    }

    public Command setModuleAngleSetpoint(Rotation2d angle){
        return new InstantCommand( () -> {
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
                this::getPose,
                this::resetOdometry,
                this::getRobotVelocity,
                (speedsRobotRelative, moduleFeedForwards) -> {
                    swerveDrive.setChassisSpeeds(speedsRobotRelative);
                },
                new PPHolonomicDriveController(
                        // Translation PID constants
                        new PIDConstants(autoXController.getP(), autoXController.getI(), autoXController.getD()),
                        // Rotation PID constants
                        new PIDConstants(autoHeadingController.getP(), autoHeadingController.getI(), autoHeadingController.getD())
                ),
                config,
                Utilities::isRedAlliance,
                this);

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
                        new Config(),
                        this, swerveDrive, 12, true),
                3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(),
                        this, swerveDrive),
                3.0, 5.0, 3.0);
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
     * @param distanceInMeters       the distance to drive in meters
     * @param speedInMetersPerSecond the speed at which to drive in meters per second
     * @return a Command that drives the swerve drive to a specific distance at a given speed
     */
    public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
        return new InstantCommand(() -> resetOdometry(new Pose2d())).andThen(run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
                .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d()) > distanceInMeters));
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

    public Command changeSwerveSpeed(double speed){
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

    @Setter
    private double elevatorHeight;

    @Entry(type = EntryType.Subscriber)
    private static double maxAcceleration = 5, heightAccelerationMultiplier = 0.1;

    @Entry(type = EntryType.Subscriber)
    private static boolean useAccelerationLimiting = true;

    private final SlewRateLimiter accelerationLimiter = new SlewRateLimiter(maxAcceleration);
    private final Timer velocityDebounce = new Timer();

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        if(useAccelerationLimiting) {
            double magnitude = Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
            double newAcceleration = accelerationLimiter.calculate(magnitude);
            double elevatorHeightInverted = MathUtil.clamp(50 - elevatorHeight, 0.01, 50) / 55;

            newAcceleration *= elevatorHeightInverted * heightAccelerationMultiplier;

            newAcceleration = MathUtil.clamp(newAcceleration, 0, magnitude);

            Dashboard.putValue("SwerveSubsystem/New Acceleration", newAcceleration);

            ChassisSpeeds newVelocity = new ChassisSpeeds();
            if (magnitude != 0)
                newVelocity = velocity.div(magnitude).times(newAcceleration);
            newVelocity.omegaRadiansPerSecond = velocity.omegaRadiansPerSecond;
            swerveDrive.drive(newVelocity.times(swerveSpeed));

            velocityDebounce.reset();
        }else{
            swerveDrive.drive(velocity.times(swerveSpeed));
        }
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
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
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
}