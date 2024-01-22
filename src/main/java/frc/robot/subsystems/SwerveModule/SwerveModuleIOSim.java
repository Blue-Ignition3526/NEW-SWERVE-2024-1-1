package frc.robot.subsystems.SwerveModule;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModuleIOSim implements SwerveModuleIO {
    // Swerve module name
    private final String m_name;

    // Current state
    private SwerveModuleState state = new SwerveModuleState();
    private double stateUpdated = Timer.getFPGATimestamp();
    private double distance = 0;

    public SwerveModuleIOSim(Object[] Arr) {
        // Set module name
        m_name = (String) Arr[7];
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.angle = state.angle.getRadians();
        inputs.speed = state.speedMetersPerSecond;
        inputs.distance = this.distance;
        this.distance += this.state.speedMetersPerSecond * (Timer.getFPGATimestamp() - this.stateUpdated);
    }

    @Override
    public void stop() {
        state = new SwerveModuleState(0, state.angle);
    }

    @Override
    public void setState(SwerveModuleState state) {
        this.state = state;
        this.stateUpdated = Timer.getFPGATimestamp();
    }

    @Override
    public String getName() {
        return this.m_name;
    }

    // Returns the current state
    public SwerveModuleState getState() {
        return state;
    }

    // Returns the current state
    public SwerveModuleState getRealState() {
        return state;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.distance, state.angle);
    }

    public void periodic() {
        // Show the target state speed and rotation in radians and degrees on the dashboard
        Logger.recordOutput("SwerveDrive/" + m_name + "/TargetSpeed", state.speedMetersPerSecond);
        Logger.recordOutput("SwerveDrive/" + m_name + "/TargetRotationRad", state.angle.getRadians());
        Logger.recordOutput("SwerveDrive/" + m_name + "/TargetRotationDeg", state.angle.getDegrees());

        // Log the state and position of the swerve module
        Logger.recordOutput("SwerveDrive/" + m_name + "/State", state);
        Logger.recordOutput("SwerveDrive/" + m_name + "/Position", getPosition());
    }
}
