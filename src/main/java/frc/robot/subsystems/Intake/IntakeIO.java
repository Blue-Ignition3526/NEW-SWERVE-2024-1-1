package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        double intakePosition;
        double rollerRpm;
    }

    public void setIntakePosition(double positionDeg);
    public Rotation2d getIntakePosition();

    public void setRollerRpm(double speedRpm);
    public void setRollerSpeed(double speed);
    public double getRollerSpeed();

    public void stopRoller();
    public void stopIntake();

    public void updateInputs(IntakeIOInputs inputs);
    public default void periodic() {}
}
