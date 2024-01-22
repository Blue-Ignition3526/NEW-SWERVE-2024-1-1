package frc.robot.subsystems.Gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon implements GyroIO {
    public Pigeon2 gyro;

    public GyroIOPigeon(int canID) {
        gyro = new Pigeon2(canID);
    }

    public double getPitch() {
        return gyro.getPitch().refresh().getValue();
    }

    public double getYaw() {
        return gyro.getPitch().refresh().getValue();
    }

    public double getRoll() {
        return gyro.getPitch().refresh().getValue();
    }

    public double getPitchVelocity() {
        return gyro.getAngularVelocityXWorld().refresh().getValue();
    }

    public double getYawVelocity() {
        return gyro.getRate();
    }

    public double getRollVelocity() {
        return gyro.getAngularVelocityZWorld().refresh().getValue();
    }

    public double getAccelerationX() {
        return gyro.getAccelerationX().refresh().getValue();
    }

    public double getAccelerationY() {
        return gyro.getAccelerationY().refresh().getValue();
    }

    public double getAccelerationZ() {
        return gyro.getAccelerationZ().refresh().getValue();
    }

    public void reset() {
        // Reset the gyro to 0 degrees
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.reset();
            } catch (Exception e) {}
        }).start();
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.pitch = getPitch();
        inputs.yaw = getYaw();
        inputs.roll = getRoll();

        inputs.pitchVelocity = getPitchVelocity();
        inputs.yawVelocity = getYawVelocity();
        inputs.rollVelocity = getRollVelocity();

        inputs.accelerationX = getAccelerationX();
        inputs.accelerationY = getAccelerationY();
        inputs.accelerationZ = getAccelerationZ();
    };
}
