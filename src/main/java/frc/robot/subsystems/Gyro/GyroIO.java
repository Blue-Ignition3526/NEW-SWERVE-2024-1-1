package frc.robot.subsystems.Gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        double pitch;
        double yaw;
        double roll;

        double pitchVelocity;
        double yawVelocity;
        double rollVelocity;

        double accelerationX;
        double accelerationY;
        double accelerationZ;
    }

    double getPitch();
    double getYaw();
    double getRoll();

    double getPitchVelocity();
    double getYawVelocity();
    double getRollVelocity();

    double getAccelerationX();
    double getAccelerationY();
    double getAccelerationZ();

    void reset();

    void updateInputs(GyroIOInputs inputs);

    default void periodic() {};
}
