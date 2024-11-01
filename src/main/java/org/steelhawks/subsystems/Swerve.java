package org.steelhawks.subsystems;


import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.steelhawks.Constants;
import org.steelhawks.Robot;
import org.steelhawks.RobotContainer;
import org.steelhawks.SwerveModule;
import org.steelhawks.lib.Limelight;
import org.steelhawks.lib.OdometryImpl;

public class Swerve extends SubsystemBase {

    public OdometryImpl odometryImpl = new OdometryImpl();
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] kSwerveModules;
    public Pigeon2 gyro;
    public Field2d field;

    /* Limelights/Cameras/Configs */
    private Limelight limelight;
    private RobotConfig config;

    private double speedMultiplier = 1;

    public void toggleMultiplier() {
        speedMultiplier = speedMultiplier == 1 ? Constants.Swerve.SLOW_MODE_MULTIPLIER : 1;
    }

    public boolean isSlowMode() {
        return speedMultiplier == Constants.Swerve.SLOW_MODE_MULTIPLIER;
    }

    private final PIDController alignPID = new PIDController(
        Constants.Swerve.autoAlignKP,
        Constants.Swerve.autoAlignKI,
        Constants.Swerve.autoAlignKD
    );

    public Swerve() {
        kSwerveModules = new SwerveModule[]{
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants),
        };

        gyro = new Pigeon2(Constants.Swerve.PIGEON_ID, Constants.PIGEON_CAN_NAME);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        field = new Field2d();

        alignPID.enableContinuousInput(0, 360);
        alignPID.setTolerance(1);

        /* Make sure to set up the robot config in Pathplanner App */
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        /* PathPlanner Configuration */
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also, optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(Constants.AutonConstants.TRANSLATION_KP, Constants.AutonConstants.TRANSLATION_KI, Constants.AutonConstants.TRANSLATION_KD), // Translation PID constants
                        new PIDConstants(Constants.AutonConstants.ROTATION_KP, Constants.AutonConstants.ROTATION_KI, Constants.AutonConstants.ROTATION_KD) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void initializePoseEstimator() {
        DriverStation.reportWarning("Initializing pose estimator", false);
        Pose2d origin;

        if (RobotContainer.alliance == DriverStation.Alliance.Red) {
            origin = Constants.Pose.Red.INITIAL_POSE;
        } else {
            origin = Constants.Pose.Blue.INITIAL_POSE;
        }

        resetModulesToAbsolute();

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.SWERVE_KINEMATICS,
            getGyroYaw(),
            getModulePositions(),
            origin,
            odometryImpl.createStdDevs(Constants.PoseConfig.POSITION_STD_DEV_X, Constants.PoseConfig.POSITION_STD_DEV_Y, Constants.PoseConfig.POSITION_STD_DEV_THETA),
            odometryImpl.createStdDevs(Constants.PoseConfig.VISION_STD_DEV_X, Constants.PoseConfig.VISION_STD_DEV_Y, Constants.PoseConfig.VISION_STD_DEV_THETA)
        );
    }

    private void resetModulesToAbsolute() {
        for (SwerveModule module : kSwerveModules) {
            module.resetToAbsolute();
        }
    }

    public Rotation2d getHeading() {
        return getRelativePose().getRotation();
    }

    public Pose2d getPose() {
        // was Constants.BlueTeamPoses.origin
        if(poseEstimator == null) return Constants.Pose.Blue.INITIAL_POSE;
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Pose2d getRelativePose() {
        if(poseEstimator == null) return Constants.Pose.Blue.INITIAL_POSE;

        if(RobotContainer.alliance == DriverStation.Alliance.Blue) {
            return poseEstimator.getEstimatedPosition();
        }
        else {
            return poseEstimator.getEstimatedPosition().relativeTo(Constants.Pose.Red.INITIAL_POSE);
        }
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : kSwerveModules){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : kSwerveModules){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public PIDController getAlignController() {
        return alignPID;
    }

    public void zeroHeading(){
        Pose2d zeroPose;
        if(RobotContainer.alliance == DriverStation.Alliance.Blue) {
            zeroPose = new Pose2d(getPose().getTranslation(), new Rotation2d());
        }
        else {
            zeroPose = new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(180));
        }
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), zeroPose);
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
        if (poseEstimator == null || limelight == null) return;

        Pose2d visionMeasurement = odometryImpl.getVisionMeasurement(limelight);
        if (visionMeasurement != null) {
            poseEstimator.addVisionMeasurement(visionMeasurement, limelight.getLimelightLatency());
        }
    }

    private int counter = 0;
    @Override
    public void periodic() {
        counter = (counter + 1) % 1000;

        if (counter % 2 == 0) { // run every 2 cycles (25 times a second)
            if (poseEstimator != null) poseEstimator.update(getGyroYaw(), getModulePositions());
        }

        if (Robot.state != Robot.RobotState.AUTON || RobotContainer.s_Autos.getUseVision() && RobotContainer.addVisionMeasurement && (counter % 3 == 0)) { // run every 16 cycles
            /* Run your limelight pose estimators here */
//            addLimelightToEstimator(limelight);
        }
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for(SwerveModule mod : kSwerveModules){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX() * speedMultiplier,
                                translation.getY() * speedMultiplier,
                                rotation * speedMultiplier,
                                getHeading()
                        )
                                : new ChassisSpeeds(
                                translation.getX() * speedMultiplier,
                                translation.getY() * speedMultiplier,
                                rotation * speedMultiplier)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for(SwerveModule mod : kSwerveModules){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void stop() {
        this.drive(new Translation2d(0, 0), 0, true, true);
    }
}

