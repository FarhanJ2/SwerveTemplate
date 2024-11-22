package org.steelhawks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    /** Enables tuning mode, which allows values to be changed on the fly in AdvantageScope */
    public static boolean TUNING_MODE = false;
    /** Enables whether simulation should play a replay of a real robot log. */
    public static boolean IN_REPLAY_MODE = false;

    public static String CANIVORE_NAME = ""; // name this
    public static String PIGEON_CAN_NAME = "";

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class SelectorConstants {
        public static final int AUTON_PORT_1 = 28;
        public static final int AUTON_PORT_2 = 29;
        public static final int AUTON_PORT_3 = 30;
    }

    public static final class Pose {
        public static final class Red {
            public static final Pose2d ORIGIN = new Pose2d(new Translation2d(16.542, 8.014), new Rotation2d(Math.PI));
        }

        public static final class Blue {
            public static final Pose2d ORIGIN = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        }
    }

    public static final class Deadbands {
        public static double DRIVE_DEADBAND = 0.1;
    }

    public static final class LED {
        public static final int LED_STRIP_LENGTH = 40;
        public static final int LED_PORT = 0;
    }

    public static class LimelightConstants {
        public static final String limelightShooter = "limelight-shooter";
        public static final String limelightArm = "limelight-arm";

        // Tracking constants
        public static final double minAreaOfTag = .1;
        public static final double maxVisionPoseError = 0.5;

        // Pipeline IDS
        public static final int limelightShooterTagPipeline = 0;
        public static final int limelightArmTagPipeline = 0;
    }

    /* You MUST set for your own robot */
    public static class PoseConfig {
        // Increase these numbers to trust your model's state estimates less.
        public static final double POSITION_STD_DEV_X = 0.1;
        public static final double POSITION_STD_DEV_Y = 0.1;
        public static final double POSITION_STD_DEV_THETA = 50; // 10

        // Increase these numbers to trust global measurements from vision less.
        public static final double VISION_STD_DEV_X = 5;
        public static final double VISION_STD_DEV_Y = 5;
        public static final double VISION_STD_DEV_THETA = Double.MAX_VALUE;
    }

    public static class AutonConstants {
        public static final double TRANSLATION_KP = 0.0;
        public static final double TRANSLATION_KI = 0.0;
        public static final double TRANSLATION_KD = 0.0;

        public static final double ROTATION_KP = 0.0;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;
    }
}
