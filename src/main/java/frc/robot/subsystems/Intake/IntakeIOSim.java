package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeIOSim implements IntakeIO {
    boolean intakeMotorStopped;
    double targetIntakePosition;
    double targetRollerRpm;

    public IntakeIOSim() {
        intakeMotorStopped = true;
        targetIntakePosition = 0;
        targetRollerRpm = 0;
    }

    public void setIntakePosition(double positionDeg) {
        this.intakeMotorStopped = false;
        this.targetIntakePosition = positionDeg;
    }

    public Rotation2d getIntakePosition() {
        return Rotation2d.fromDegrees(targetIntakePosition);
    }
    
    public void setRollerRpm(double speedRpm) {
        this.targetRollerRpm = speedRpm;
    }

    public void setRollerSpeed(double speed) {
        this.targetRollerRpm = 0;
    }

    public double getRollerSpeed() {
        return this.targetRollerRpm;
    }

    public void stopRoller() {
        this.targetRollerRpm = 0;
    }

    public void stopIntake() {
        this.intakeMotorStopped = true;
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakePosition = getIntakePosition().getDegrees();
        inputs.rollerRpm = getRollerSpeed();
    }

    public void periodic() {
        Logger.recordOutput("Intake/IntakePositionDeg", getIntakePosition().getDegrees());
        Logger.recordOutput("Intake/IntakePositionRad", getIntakePosition().getRadians());

        Logger.recordOutput("Intake/RollerSpeedRpm", getRollerSpeed());
    }
}
