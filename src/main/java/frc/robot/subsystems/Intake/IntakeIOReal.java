package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.Intake;

public class IntakeIOReal implements IntakeIO {
    private CANSparkMax intakeMotor;
    private SparkPIDController intakeMotorPIDController;
    private RelativeEncoder intakeMotorEncoder;

    private CANSparkMax rollerMotor;
    private SparkPIDController rollerMotorPIDController;
    private RelativeEncoder rollerMotorEncoder;

    private double targetIntakePosition;
    private boolean intakeMotorStopped;
    private double targetRollerRpm;

    public IntakeIOReal(int intakeMotorId, int rollerMotorId) {
        // Create Intake Motor controller
        this.intakeMotor = new CANSparkMax(intakeMotorId, MotorType.kBrushless);

        // Get the motor's encoder
        this.intakeMotorEncoder = this.intakeMotor.getEncoder();

        // Configure the encoder conversions
        this.intakeMotorEncoder.setPositionConversionFactor(Intake.kIntakeMotorEncoder_RotationToDegrees);
        
        // Get the intake motor's Spark Max's PID Controller
        this.intakeMotorPIDController = this.intakeMotor.getPIDController();

        // Configure the intake motor's PID Controller
        this.intakeMotorPIDController.setP(Intake.IntakeMotorPIDConstants.kP);
        this.intakeMotorPIDController.setI(Intake.IntakeMotorPIDConstants.kI);
        this.intakeMotorPIDController.setD(Intake.IntakeMotorPIDConstants.kD);

        // Create Roller Motor Controller
        this.rollerMotor = new CANSparkMax(rollerMotorId, MotorType.kBrushless);

        // Get the motor's encoder
        this.rollerMotorEncoder = this.rollerMotor.getEncoder();

        // Get roller motor's Spark Max's PID Controller
        this.rollerMotorPIDController = this.rollerMotor.getPIDController();

        // Configure intake motor's PID Controller
        this.rollerMotorPIDController.setP(Intake.RollerMotorPIDConstants.kP);
        this.rollerMotorPIDController.setI(Intake.RollerMotorPIDConstants.kI);
        this.rollerMotorPIDController.setD(Intake.RollerMotorPIDConstants.kD);
    }

    public void setIntakePosition(double positionDeg) {
        this.intakeMotorStopped = false;
        this.targetIntakePosition = positionDeg;
    }

    public Rotation2d getIntakePosition() {
        return Rotation2d.fromDegrees(intakeMotorEncoder.getPosition());
    }
    
    public void setRollerRpm(double speedRpm) {
        this.targetRollerRpm = speedRpm;
    }

    public void setRollerSpeed(double speed) {
        this.targetRollerRpm = 0;
        this.rollerMotor.set(speed);
    }

    public double getRollerSpeed() {
        return rollerMotorEncoder.getVelocity();
    }

    public void stopRoller() {
        this.targetRollerRpm = 0;
        this.rollerMotor.set(0);
    }

    public void stopIntake() {
        this.intakeMotorStopped = true;
        this.intakeMotor.set(0);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakePosition = getIntakePosition().getDegrees();
        inputs.rollerRpm = getRollerSpeed();
    }

    public void periodic() {
        if (!intakeMotorStopped) intakeMotorPIDController.setReference(targetIntakePosition, ControlType.kPosition);
        if (Math.abs(targetRollerRpm) > 0) rollerMotorPIDController.setReference(targetRollerRpm, ControlType.kVelocity);

        Logger.recordOutput("Intake/IntakePositionDeg", getIntakePosition().getDegrees());
        Logger.recordOutput("Intake/IntakePositionRad", getIntakePosition().getRadians());

        Logger.recordOutput("Intake/RollerSpeedRpm", getRollerSpeed());
    }
}
