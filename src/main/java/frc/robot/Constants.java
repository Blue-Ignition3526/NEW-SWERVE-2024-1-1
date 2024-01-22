// Archivo donde se guardan las constantes del robot
////////////////////////////////////////////////////
// File where the robot constants are stored

package frc.robot;

import org.team3526.lib.util.Conversions;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Constants {

  public static final class Intake {
    public static final double kMechanismWidth = 20;
    public static final double kMechanismheight = 40;

    public static final int intakeMotor = 31;
    public static final int rollerMotor = 32;

    public static final double kIntakeMotorEncoder_RotationToDegrees = (1 / 16) * 360; // Conversion Rotaciones a grados

    public static final double kRollerInSpeedRpm = 20;
    public static final double kRollerOutSpeedRpm = -20;

    public static final class IntakeMotorPIDConstants {
      public static final double kP = 0.01;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class RollerMotorPIDConstants {
      public static final double kP = 0.001;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }
  }

  public final static class Vision {
    public static final String kLimelightName = "limelight";

    public static final String kLimelightCameraName = "OV5647";
    // TODO: change camera names
    public static final String kFrontCameraName = "Front";
    public static final String kBackCameraName = "Back";
    public static final String kLeftCameraName = "Left";
    public static final String kRightCameraName = "Right";
    

    public static final boolean[] kEnabledCameras = {
      false, // Limelight
      false, // Front
      false, // Back
      false, // Left
      false // Right
    };

    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToLime = new Transform3d(
      new Translation3d(Constants.Swerve.Physical.kTrackWidth / 2, 0.28, Constants.Swerve.Physical.kTrackWidth / 2),
      new Rotation3d(0, 0, 0)
    );

    // TODO: change camera transforms
    public static final Transform3d kRobotToFrontCamera = new Transform3d(
      new Translation3d(0, 0, 0),
      new Rotation3d(0, 0, 0)
    );

    public static final Transform3d kRobotToBackCamera = new Transform3d(
      new Translation3d(0, 0, 0),
      new Rotation3d(0, 0, 0)
    );

    public static final Transform3d kRobotToLeftCamera = new Transform3d(
      new Translation3d(0, 0, 0),
      new Rotation3d(0, 0, 0)
    );

    public static final Transform3d kRobotToRightCamera = new Transform3d(
      new Translation3d(0, 0, 0),
      new Rotation3d(0, 0, Math.PI)
    );

    public static final PIDController m_activeTrackPIDController = new PIDController(0.05, 0, 0.005);
    {
      SmartDashboard.putData("SwerveDrive/ActiveTrackPIDController", m_activeTrackPIDController);
    }

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect correction rates
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8); // Experimentally determined
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1); // Experimentally determined

    public final static class AprilTags {
      // Tag #10 is for the amp, we are using the amp as our test, in the future we will use the speaker
      public static final int kSpeakerTagID = 4;
    }
  }

  public static final class Field {
    public static final double kFieldLength = 15.98;
    public static final double kFieldWidth = 8.21;

    public static final Pose2d kRedStartPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0)); //! TODO: change start pose
    public static final Pose2d kBlueStartPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

    public static final Pose2d kStartPose = kRedStartPose;
  }

  public final static class Swerve {

    public static final class Module {
      public static final double kWheelDiameterMeters = Conversions.inchToM(4.0); // 4 inches
      public static final double kDriveMotorGearRatio = 1.0 / 6.12; // 6.12:1 Drive
      public static final double kTurningMotorGearRatio = 1.0 / 12.8; // 12.8:1 Steering

      public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI; // Conversion Rotaciones a Metros
      public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0; // Conversion RPM a Metros por Segundo

      public static final double kTurningEncoder_RotationToRadian = kTurningMotorGearRatio * 2.0 * Math.PI; // Conversion Rotaciones a Radianes
      public static final double kTurningEncoder_RPMToRadianPerSecond = kTurningEncoder_RotationToRadian / 60.0; // Conversion RPM a Radianes por Segundo

      public static final double kGlobalTurningOffsetRad = Math.toRadians(180);

      public static final ProfiledPIDController kTurningPIDController = new ProfiledPIDController(
        0.1,
        0,
        0,
        new TrapezoidProfile.Constraints(0.5, 0.075)
      );
      {
        kTurningPIDController.enableContinuousInput(0, 2 * Math.PI);
        SmartDashboard.putData("SwerveDrive/TurningPIDController", kTurningPIDController);
      }
    }

    public final static class Physical {
        //582.5 mm LEFT RIGHT
        //583.0 mm FRONT BACK
        public static final double kTrackWidth = Conversions.mmToM(582.5); // Distance between left and right wheels
        public static final double kWheelBase = Conversions.mmToM(583.0); // Distance between front and back wheels

        public static final Translation2d m_frontLeftLocation = new Translation2d(kWheelBase/2, -kTrackWidth/2); // Front Left Wheel Location
        public static final Translation2d m_frontRightLocation = new Translation2d(kWheelBase/2, kTrackWidth/2); // Front Right Wheel Location
        public static final Translation2d m_backLeftLocation = new Translation2d(-kWheelBase/2, -kTrackWidth/2); // Back Left Wheel Location
        public static final Translation2d m_backRightLocation = new Translation2d(-kWheelBase/2, kTrackWidth/2); // Back Right Wheel Location

        public static final double kMaxSpeedMetersPerSecond = 15.0; // Maxima Velocidad en Metros por Segundo
        //public static final double kMaxSpeedMetersPerSecond = 5.0; // Maxima Velocidad en Metros por Segundo
        public static final double kMaxAngularSpeedRadiansPerSecond = 3 * 2.0 * Math.PI; // Maxima Velocidad Angular en Radianes por Segundo

        public static final double kMaxAccelerationUnitsPerSecond = 15; // Maxima Aceleracion
        public static final double kMaxAngularAccelerationUnitsPerSecond = 1.5 * Math.PI; // Maxima Aceleracion Angular

        public static final double kTeleopMaxSpeedMetersPerSecond = kMaxSpeedMetersPerSecond / 4.0; // Maxima Velocidad en Metros por Segundo
        public static final double kTeleopMaxAngularSpeedRadiansPerSecond = kMaxAngularSpeedRadiansPerSecond / 4.0; // Maxima Velocidad Angular en Radianes por Segundo

        public static final double kTeleopMaxAccelerationUnitsPerSecond = 2.0; // Maxima Aceleracion
        public static final double kTeleopMaxAngularAccelerationUnitsPerSecond = 1.0; // Maxima Aceleracion Angular

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(kTeleopMaxAngularSpeedRadiansPerSecond, kTeleopMaxAngularAccelerationUnitsPerSecond);

        public static final SwerveDriveKinematics m_swerveDriveKinematics = new SwerveDriveKinematics(
          Constants.Swerve.Physical.m_frontLeftLocation,
          Constants.Swerve.Physical.m_frontRightLocation,
          Constants.Swerve.Physical.m_backLeftLocation,
          Constants.Swerve.Physical.m_backRightLocation
        );


      }

      public static final class Autonomous {
        public static final PIDConstants kTranslatePIDConstants = new PIDConstants(0.1, 0.0, 0.0);
        public static final PIDConstants kRotatePIDConstants = new PIDConstants(0.1, 0.0, 0.0);

        public static final double kMaxSpeedMetersPerSecond = 1.0; // Maxima Velocidad en Metros por Segundo
      }

    public final static class Motors {
      //! OFFSETS ARE CALCULATED AS THE -(THE DIFFERENCE OF SWERVE ABSOLUTE ENCODER "0" and ABSOLUTE ENCODER 0 ---IN RADIANS---)
      public static final Object[] kFrontLeftVars = { 
        Math.toRadians(-74.970703125), // Offset
        true, // Inverted
        11, // Absolute Encoder ID
        22, // Drive Motor ID
        21, // Turning Motor ID
        false, // Drive Motor Inverted
        true, // Turning Motor Inverted
        "Front Left" // Name
      };
      public static final Object[] kFrontRightVars = {
        Math.toRadians(-38.583984375), // Offset
        true, // Inverted
        12, // Absolute Encoder ID
        24, // Drive Motor ID
        23, // Turning Motor ID
        false, // Drive Motor Inverted
        true, // Turning Motor Inverted
        "Front Right" // Name
      };
      public static final Object[] kBackLeftVars = {
        Math.toRadians(-56.953), // Offset
        true, // Inverted
        13, // Absolute Encoder ID
        26, // Drive Motor ID
        25, // Turning Motor ID
        true, // Drive Motor Inverted
        true, // Turning Motor Inverted
        "Back Left" // Name
      };
      public static final Object[] kBackRightVars = {
        Math.toRadians(-105.46875), // Offset
        true, // Inverted
        14, // Absolute Encoder ID
        28, // Turning Motor ID
        27, // Drive Motor ID
        false, // Drive Motor Inverted
        true, // Turning Motor Inverted
        "Back Right" // Name
      };
    }

    public static final class PoseEstimation {
      public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.5, 0.5, 0.5); // TODO: find the Swerve module standard deviatioins
      // SwervePoseEstimator wont work without them
    }
  }

  public static final class Operator { // Operator Controllers and Data
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kDeadzone = 0.1;
  }

}
