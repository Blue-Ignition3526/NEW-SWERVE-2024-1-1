package frc.robot.subsystems.SwerveDrive;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule.SwerveModule;

public class SwerveDriveIOSim implements SwerveDriveIO {
    // Create all swerve modules
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    // Create a swerve drive odometry instance to calculate robot position
    private final SwerveDriveOdometry m_odometry;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
    private double heading = 0;
    private double speedsUpdated = Timer.getFPGATimestamp();
    private boolean drivingRobotRelative = false;

    public SwerveDriveIOSim(SwerveModule m_frontLeft, SwerveModule m_frontRight, SwerveModule m_backLeft, SwerveModule m_backRight) {
        this.m_frontLeft = m_frontLeft;
        this.m_frontRight = m_frontRight;
        this.m_backLeft = m_backLeft;
        this.m_backRight = m_backRight;

        this.m_odometry = new SwerveDriveOdometry(Constants.Swerve.Physical.m_swerveDriveKinematics, getRotation2d(), new SwerveModulePosition[]{
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
    }

    /**
     * Get the current ROBOT RELATIVE ChassisSpeeds
     * @return ChassisSpeeds
     */
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        if (this.drivingRobotRelative) {
            return this.m_chassisSpeeds;
        } else {
            return ChassisSpeeds.fromFieldRelativeSpeeds(m_chassisSpeeds, getRotation2d());
        }
    }

    /**
     * Configure the auto builder
     * @param swerveDrive The swerve drive subsystem
     */
    public void configureAutoBuilder(SwerveDrive swerveDrive) {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPosition,
            this::getRobotRelativeChassisSpeeds,
            this::driveRobotRelative,
            new HolonomicPathFollowerConfig(
                Constants.Swerve.Autonomous.kTranslatePIDConstants,
                Constants.Swerve.Autonomous.kRotatePIDConstants,
                Constants.Swerve.Autonomous.kMaxSpeedMetersPerSecond,
                Constants.Swerve.Physical.kTrackWidth / 2,
                new ReplanningConfig()
            ),
            () -> {
                if (DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == Alliance.Red;
                return false;
            },
            swerveDrive
        );
    }



    /**
     * Tries to update the odometry (if it fails, it will print the error and still return the odometry)
     * @return SwerveDriveOdometry
     */
    public SwerveDriveOdometry getOdometry() {
        try {
            m_odometry.update(
                getRotation2d(),
                new SwerveModulePosition[]{
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
                }
            );
        } catch (Exception e) {
            System.out.println("Error updating odometry: " + e);
        }
        return m_odometry;
    }

    /**
     * Get the current pose of the robot
     * @return Pose2d
     */
    public Pose2d getPose() {
        return getOdometry().getPoseMeters();
    }

    /**
     * Reset the pose of the robot
     * @param Pose2d
     */
    public void resetPosition(Pose2d pose) {
        m_odometry.resetPosition(this.getRotation2d(), new SwerveModulePosition[]{
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }, pose);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.Physical.kMaxSpeedMetersPerSecond);
        m_frontLeft.setState(desiredStates[0]);
        m_frontRight.setState(desiredStates[1]);
        m_backLeft.setState(desiredStates[2]);
        m_backRight.setState(desiredStates[3]);
    }

    public void stopModules() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    public void updateInputs(SwerveDriveIOInputs inputs) {
        inputs.frontLeftAngle = this.m_frontLeft.getState().angle.getRadians();
        inputs.frontLeftSpeed = this.m_frontLeft.getState().speedMetersPerSecond;

        inputs.frontRightAngle = this.m_frontRight.getState().angle.getRadians();
        inputs.frontRightSpeed = this.m_frontRight.getState().speedMetersPerSecond;

        inputs.backLeftAngle = this.m_backLeft.getState().angle.getRadians();
        inputs.backLeftSpeed = this.m_backLeft.getState().speedMetersPerSecond;

        inputs.backRightAngle = this.m_backRight.getState().angle.getRadians();
        inputs.backRightSpeed = this.m_backRight.getState().speedMetersPerSecond;

        inputs.gyroAngle = getRotation2d().getDegrees();

        inputs.xSpeed = m_chassisSpeeds.vxMetersPerSecond;
        inputs.ySpeed = m_chassisSpeeds.vyMetersPerSecond;
        inputs.rotSpeed = m_chassisSpeeds.omegaRadiansPerSecond;

        this.heading += m_chassisSpeeds.omegaRadiansPerSecond * (Timer.getFPGATimestamp() - this.speedsUpdated);
    }

    public void setM_chassisSpeeds(ChassisSpeeds speeds) {
        this.m_chassisSpeeds = speeds;
        this.speedsUpdated = Timer.getFPGATimestamp();
    }

    public ChassisSpeeds getM_chassisSpeeds() {
        return this.m_chassisSpeeds;
    }

    public Rotation2d getRotation2d() {
        // Calculate the new robot heading angle using the angle theta provided 
        return new Rotation2d(this.heading);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }

    /**
     * Drive the robot using the given speeds (calculate the states for each swerve module and apply it)
     * @param speeds The speeds to drive at (Check `ChassisSpeeds` for more info)
     */
    public void drive(ChassisSpeeds speeds) {
        this.m_chassisSpeeds = speeds;
        SwerveModuleState[] m_moduleStates = Constants.Swerve.Physical.m_swerveDriveKinematics.toSwerveModuleStates(speeds);
        this.setModuleStates(m_moduleStates);
    }

    /**
     * Drive the robot relative to the field
     * @param xSpeed X speed in meters per second
     * @param ySpeed Y speed in meters per second
     * @param rotSpeed Rotation speed in radians per second
     */
    public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = false;
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, this.getRotation2d()));
    }

    /**
     * Drive the robot relative to the field
     * @param speeds The speeds to drive at (Check `ChassisSpeeds` for more info)
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = false;
        this.drive(speeds);
    }

    /**
     * Drive the robot relative to the robot's current position
     * @param xSpeed X speed in meters per second
     * @param ySpeed Y speed in meters per second
     * @param rotSpeed Rotation speed in radians per second
     */
    public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = true;
        this.drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
    }

    /**
     * Drive the robot relative to the field
     * @param speeds The speeds to drive at (Check `ChassisSpeeds` for more info)
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = true;
        this.drive(speeds);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }

    public void periodic() {
        Logger.recordOutput("SwerveDrive/RobotHeadingRad", this.getRotation2d().getRadians());
        Logger.recordOutput("SwerveDrive/RobotHeadingDeg", this.getRotation2d().getDegrees());
        
        Logger.recordOutput("SwerveDrive/RobotPose", this.getPose());
        Logger.recordOutput("SwerveDrive/RobotX", this.getPose().getX());
        Logger.recordOutput("SwerveDrive/RobotY", this.getPose().getY());

        Logger.recordOutput("SwerveDrive/RobotRelative", this.drivingRobotRelative);
        Logger.recordOutput("SwerveDrive/RobotSpeeds", this.getRobotRelativeChassisSpeeds());

        Logger.recordOutput("SwerveDrive/RobotSpeedsX", this.getRobotRelativeChassisSpeeds().vxMetersPerSecond);
        Logger.recordOutput("SwerveDrive/RobotSpeedsY", this.getRobotRelativeChassisSpeeds().vyMetersPerSecond);
        Logger.recordOutput("SwerveDrive/RobotSpeedsRot", this.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond);

        Logger.recordOutput("SwerveDrive/SwerveModuleStates", this.getModuleStates());
    }
}
