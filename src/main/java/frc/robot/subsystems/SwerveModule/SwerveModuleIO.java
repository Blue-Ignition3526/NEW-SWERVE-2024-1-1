package frc.robot.subsystems.SwerveModule;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        double angle;
        double speed;
        double distance;
    }

    public void stop();
    public void setState(SwerveModuleState state);
    public String getName();
    public SwerveModuleState getState();
    public SwerveModuleState getRealState();
    public SwerveModulePosition getPosition();
    public void updateInputs(SwerveModuleIOInputs inputs);
    public default void periodic() {}
}
