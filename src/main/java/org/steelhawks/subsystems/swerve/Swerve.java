package org.steelhawks.subsystems.swerve;


import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.lib.Limelight;
import org.steelhawks.lib.OdometryImpl;

import java.util.Arrays;

public class Swerve extends SubsystemBase {

    public final OdometryImpl odometryImpl = new OdometryImpl();
    public SwerveDrivePoseEstimator mPoseEstimator;
    public SwerveModule[] mSwerveModules;
    public Pigeon2 mGyro;
    public Field2d field;

    private double speedMultiplier = 1;

    public void toggleMultiplier() {
        speedMultiplier = speedMultiplier == 1 ? KSwerve.SLOW_MODE_MULTIPLIER : 1;
    }

    public boolean isSlowMode() {
        return speedMultiplier == KSwerve.SLOW_MODE_MULTIPLIER;
    }

    /* Limelights/Cameras */
    private Limelight limelight;

    private final PIDController alignPID = new PIDController(
        KSwerve.autoAlignKP,
        KSwerve.autoAlignKI,
        KSwerve.autoAlignKD
    );

    private final static Swerve INSTANCE = new Swerve();

    public static Swerve getInstance() {
        return INSTANCE;
    }

    private Swerve() {
        if (RobotBase.isReal()) {
            mSwerveModules = new SwerveModule[]{
                new SwerveModule(0, new RealModule(0, KSwerve.Mod0.constants)),
                new SwerveModule(1, new RealModule(1, KSwerve.Mod1.constants)),
                new SwerveModule(2, new RealModule(2, KSwerve.Mod2.constants)),
                new SwerveModule(3, new RealModule(3, KSwerve.Mod3.constants)),
            };
        } else {
            mSwerveModules = new SwerveModule[]{
                new SwerveModule(0, new SimModule()),
                new SwerveModule(1, new SimModule()),
                new SwerveModule(2, new SimModule()),
                new SwerveModule(3, new SimModule()),
            };
        }


        mGyro = new Pigeon2(KSwerve.PIGEON_ID, Constants.PIGEON_CAN_NAME);
        mGyro.getConfigurator().apply(new Pigeon2Configuration());
        mGyro.setYaw(0);

        field = new Field2d();

        alignPID.enableContinuousInput(0, 360);
        alignPID.setTolerance(1);

        /* PathPlanner Configuration */
        AutoBuilder.configureHolonomic(
            this::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(
                                Constants.AutonConstants.TRANSLATION_KP,
                                Constants.AutonConstants.TRANSLATION_KI,
                                Constants.AutonConstants.TRANSLATION_KD
                        ),
                        new PIDConstants(
                                Constants.AutonConstants.ROTATION_KP,
                                Constants.AutonConstants.ROTATION_KI,
                                Constants.AutonConstants.ROTATION_KD
                        ),
                        4.3, KSwerve.TRACK_WIDTH / Math.sqrt(2),
                        new ReplanningConfig()
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this
        );
    }

    public void initializePoseEstimator() {
        DriverStation.reportWarning("Initializing pose estimator", false);
        Pose2d origin;

        if (RobotContainer.getAlliance() == DriverStation.Alliance.Red) {
            origin = Constants.Pose.Red.ORIGIN;
        } else {
            origin = Constants.Pose.Blue.ORIGIN;
        }

        resetModulesToAbsolute();

        mPoseEstimator = new SwerveDrivePoseEstimator(
            KSwerve.SWERVE_KINEMATICS,
            getGyroYaw(),
            getModulePositions(),
            origin,
            odometryImpl.createStdDevs(Constants.PoseConfig.POSITION_STD_DEV_X, Constants.PoseConfig.POSITION_STD_DEV_Y, Constants.PoseConfig.POSITION_STD_DEV_THETA),
            odometryImpl.createStdDevs(Constants.PoseConfig.VISION_STD_DEV_X, Constants.PoseConfig.VISION_STD_DEV_Y, Constants.PoseConfig.VISION_STD_DEV_THETA)
        );
    }

    private void resetModulesToAbsolute() {
        for (SwerveModule module : mSwerveModules) {
            module.resetToAbsolute();
        }
    }

    public Rotation2d getHeading() {
        return getRelativePose().getRotation();
    }

    public Pose2d getPose() {
        if(mPoseEstimator == null) return new Pose2d(new Translation2d(0, 0), new Rotation2d());
        return mPoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        mPoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Pose2d getRelativePose() {
        if(mPoseEstimator == null) return Constants.Pose.Blue.ORIGIN;

        if(RobotContainer.getAlliance() == DriverStation.Alliance.Blue) {
            return mPoseEstimator.getEstimatedPosition();
        }
        else {
            return mPoseEstimator.getEstimatedPosition().relativeTo(Constants.Pose.Red.ORIGIN);
        }
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(mGyro.getYaw().getValue());
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return KSwerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveModules){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveModules){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public PIDController getAlignController() {
        return alignPID;
    }

    public void zeroHeading(){
        Pose2d zeroPose;
        if(RobotContainer.getAlliance() == DriverStation.Alliance.Blue) {
            zeroPose = new Pose2d(getPose().getTranslation(), new Rotation2d());
        }
        else {
            zeroPose = new Pose2d(mPoseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(180));
        }

        setPose(zeroPose);
    }

    public double calculateTurnAngle(Pose2d target, double robotAngle) {
        double tx = target.getX();
        double ty = target.getY();
        double rx = getRelativePose().getX();
        double ry = getRelativePose().getY();

        double requestedAngle = Math.atan((ty - ry) / (tx - rx)) * (180 / Math.PI);
        double calculatedAngle = (180 - robotAngle + requestedAngle);

        return ((calculatedAngle + 360) % 360);
    }

    private void addLimelightToEstimator(Limelight limelight) {
        if (mPoseEstimator == null || limelight == null) return;

        Pose2d visionMeasurement = odometryImpl.getVisionMeasurement(limelight);
        if (visionMeasurement != null) {
            mPoseEstimator.addVisionMeasurement(visionMeasurement, limelight.getLimelightLatency());
        }
    }

    private double getTotalVoltage() {
        return Arrays.stream(mSwerveModules) // convert array to stream
            .mapToDouble(SwerveModule::getVoltage) // map each module to its voltage
            .sum(); // sum up all voltages
    }


    private int counter = 0;
    @Override
    public void periodic() {
        counter = (counter + 1) % 1000;

        if (counter % 2 == 0) { // run every 2 cycles (25 times a second)
            if (mPoseEstimator != null) mPoseEstimator.update(getGyroYaw(), getModulePositions());

            for (SwerveModule mod : mSwerveModules) {
                mod.updateInputs(mod.getState(), false);
            }

            Logger.recordOutput("swerve/pose", getPose());
            Logger.recordOutput("swerve/voltage", getTotalVoltage());
            Logger.recordOutput("swerve/modState", getModuleStates());
        }

        if (Robot.getState() != Robot.RobotState.AUTON || Autos.getInstance().getUseVision() && RobotContainer.addVisionMeasurement && (counter % 3 == 0)) { // run every 16 cycles
            /* Run your limelight pose estimators here */
//            addLimelightToEstimator();
        }


        field.setRobotPose(getPose());
        SmartDashboard.putData("swerve/field",  field);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = KSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, KSwerve.MAX_SPEED);

        for(SwerveModule mod : mSwerveModules){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                KSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX() * speedMultiplier,
                                translation.getY() * speedMultiplier,
                                rotation * speedMultiplier,
                                getHeading()
                        ) : new ChassisSpeeds(
                                translation.getX() * speedMultiplier,
                                translation.getY() * speedMultiplier,
                                rotation * speedMultiplier
                        )
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, KSwerve.MAX_SPEED);

        for(SwerveModule mod : mSwerveModules){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void stop() {
        this.drive(new Translation2d(0, 0), 0, true, true);
    }
}

