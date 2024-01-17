package frc.robot.subsystems.SwerveDrive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveDriveIO {
    @AutoLog
    public class SwerveDriveIOInputs {
        double frontLeftAngle;
        double frontLeftSpeed;

        double frontRightAngle;
        double frontRightSpeed;

        double backLeftAngle;
        double backLeftSpeed;

        double backRightAngle;
        double backRightSpeed;

        double gyroAngle;

        double xSpeed;
        double ySpeed;
        double rotSpeed;
    }

    public Rotation2d getRotation2d();
    public SwerveModuleState[] getModuleStates();
    public SwerveDriveOdometry getOdometry();
    public Pose2d getPose();
    public void resetPosition(Pose2d pose);
    public ChassisSpeeds getRobotRelativeChassisSpeeds();
    public void setModuleStates(SwerveModuleState[] desiredStates);
    public void stopModules();
    public void drive(ChassisSpeeds speeds);
    public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed);
    public void driveFieldRelative(ChassisSpeeds speeds);
    public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed);
    public void driveRobotRelative(ChassisSpeeds speeds);
    public void configureAutoBuilder(SwerveDrive swerveDrive);
    public void updateInputs(SwerveDriveIOInputs inputs);
    public default void periodic() {}
}
