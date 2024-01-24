package frc.robot.subsystems.SwerveModule;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class SwerveModuleIOSparkMaxPID implements SwerveModuleIO {
    /**
     * Drive motor controller
     */
    private final CANSparkMax m_driveMotor;

    /**
     * Turning motor controller
     */
    private final CANSparkMax m_turningMotor;


    /**
     * Turning motor encoder
     */
    private final RelativeEncoder m_turningMotorEncoder;

    /**
     * Drive motor encoder
     */
    private final RelativeEncoder m_driveMotorEncoder;

    /**
     * Turning motor PID Controller
     */
    private final SparkPIDController m_turningMotorPIDController;

    /**
     * Drive motor PID Controller
     */
    private final SparkPIDController m_driveMotorPIDController;


    /**
     * Absolute CANcoder
     */
    private final CANcoder m_turningAbsoluteEncoder; 


    /**
     * Turning encoder offset in radians
     */
    private final double m_turningEncoderOffsetRad;

    /**
     * Whether the absolute encoder is inverted
     */
    private final boolean m_turningAbsoluteEncoderInverted;

    /**
     * Swerve module name
     */
    private final String m_name;

    /**
     * Swerve module state
     */
    private SwerveModuleState state = new SwerveModuleState();

    /**
     * Timer for resetting the module's position
     */
    private Timer resetAngleTimer = new Timer();

    public SwerveModuleIOSparkMaxPID(Object[] Arr) {
        this.m_turningEncoderOffsetRad = (double)Arr[0]; 
        this.m_turningAbsoluteEncoderInverted = (boolean)Arr[1];
        this.m_turningAbsoluteEncoder = new CANcoder((int)Arr[2]);

        this.m_driveMotor = new CANSparkMax((int)Arr[3], MotorType.kBrushless);
        this.m_turningMotor = new CANSparkMax((int)Arr[4], MotorType.kBrushless);

        this.m_driveMotor.setInverted((boolean)Arr[5]);
        this.m_turningMotor.setInverted((boolean)Arr[6]);

        this.m_name = (String) Arr[7];

        // Drive Motor Encoder
        this.m_driveMotorEncoder = m_driveMotor.getEncoder();
        this.m_driveMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kDriveEncoder_RotationToMeter); 
        this.m_driveMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kDriveEncoder_RPMToMeterPerSecond);

        // Turn Motor Encoder
        this.m_turningMotorEncoder = m_turningMotor.getEncoder();
        this.m_turningMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kTurningEncoder_RotationToRadian); 
        this.m_turningMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kTurningEncoder_RPMToRadianPerSecond);

        // Turning absolute encoder
        this.m_turningAbsoluteEncoder.getConfigurator().apply(
            new MagnetSensorConfigs()
            .withMagnetOffset((m_turningEncoderOffsetRad + Constants.Swerve.Module.kGlobalTurningOffsetRad) / (2 * Math.PI))
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        );

        // Drive motor PID Controller
        this.m_driveMotorPIDController = m_driveMotor.getPIDController();
        this.m_driveMotorPIDController.setP(0.175);
        this.m_driveMotorPIDController.setI(0.0000000005);

        // Turn motor PID Controller
        this.m_turningMotorPIDController = m_turningMotor.getPIDController();
        this.m_turningMotorPIDController.setP(0.5);

        resetMotorEncoders();
    }

    /**
     * Return the absolute encoder position in radians
     */
    public double getAbsoluteEncoderRad() {
        return ((m_turningAbsoluteEncoder.getPosition().refresh().getValue() * 2 * Math.PI) % (2 * Math.PI)) * (m_turningAbsoluteEncoderInverted ? -1.0 : 1.0);
    }

    /**
     * Reset the motor encoder
     */
    void resetDriveMotorEncoder() {
        m_driveMotorEncoder.setPosition(0);
    }

    /**
     * Reset the motor encoder
     */
    void resetTurningMotorEncoder() {
        m_turningMotorEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
     * Reset the motor encoders
     * This function is used to reset the motor encoders to the absolute encoder value when initializing
     */
    private void resetMotorEncoders() {
        resetDriveMotorEncoder();
        resetTurningMotorEncoder();
    }

    /**
     * Get the turning motor encoder position in radians
     * @return double
     */
    public double getTurningEncoderPositionRad() {
        // Return the motor encoder position in radians
        return this.m_turningMotorEncoder.getPosition();
    }

    /**
     * Stop the swerve module (Set the speed and rotation speed to 0)
     */
    public void stop() {
        m_driveMotor.set(0);
        m_turningMotor.set(0);
    }

    /**
     * Set the state of the swerve module (Speed and rotation)
     * @param state SwerveModuleState
     */
    public void setState(SwerveModuleState state) {
        // Check if the speed is meaningful
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
    
        // Optimize the state
        state = SwerveModuleState.optimize(state, new Rotation2d(getTurningEncoderPositionRad()));

        // Set the state
        this.state = state;
    
        // Set the drive motor speed
        // The speed is given in meters per second, so we need to convert from [-1, 1]
        // ! PAST IMPLEMENTATION (NOT ACCURATE)
        // m_driveMotor.set(optimizedState.speedMetersPerSecond / Constants.Swerve.Physical.kMaxSpeedMetersPerSecond);
        m_driveMotorPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        // Set the turning motor speed
        // The speed is not given to us, rather the angle we want to turn to
        // So we need to calculate the difference between the current angle and the target angle
        // Then we use the PID controller to calculate the speed we need to turn at
        // ! PAST IMPLEMENTATION (NOT ACCURATE)
        // m_turningMotor.set(Constants.Swerve.Module.kTurningPIDController.calculate(getTurningEncoderPositionRad(), optimizedState.angle.getRadians()));

        // TODO: test this
        // Ternary checks if the difference between the current position and the desired position is more than 90º if it is adds 360º
        // This is done to combat the problem where when surpassing 360º the desired state would give < 360º so the wheel would not take the shortest path
        m_turningMotorPIDController.setReference(Math.abs(state.angle.getRadians() - getTurningEncoderPositionRad()) > Math.PI / 2 ? Math.PI * 2 + state.angle.getRadians() : state.angle.getRadians(), ControlType.kPosition);
        Logger.recordOutput("SwerveDrive/" + m_name + "/PIDAngle", Math.abs(state.angle.getRadians() - getTurningEncoderPositionRad()) > Math.PI / 2 ? Math.PI * 2 + state.angle.getRadians() : state.angle.getRadians());
    }

    /**
     * Get the name of the swerve module
     */
    public String getName() {
        return m_name;
    }
    
    /**
     * Get the state of the swerve module (Speed and rotation)
     * @return SwerveModuleState
     */
    public SwerveModuleState getState() {
        return state;
    }

    /**
     * Returns the real wheel state
     * @return SwerveModuleState
     */
    public SwerveModuleState getRealState() {
        return new SwerveModuleState(m_driveMotorEncoder.getVelocity(), new Rotation2d(getTurningEncoderPositionRad()));
    }

    /**
     * Get the position of the swerve module (Displacement and rotation)
     * @return SwerveModulePosition
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveMotorEncoder.getPosition(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Set the angle and speed of the swerve module
        inputs.angle = getAbsoluteEncoderRad();
        inputs.speed = m_driveMotorEncoder.getVelocity();
        inputs.distance = m_driveMotorEncoder.getPosition();
    }

    public void periodic() {
        // Showt the absolute encoder value on the dashboard
        Logger.recordOutput("SwerveDrive/" + m_name + "/AbsEncoderRad", getAbsoluteEncoderRad());

        // Show the absolute encoder value in degrees on the dashboard
        Logger.recordOutput("SwerveDrive/" + m_name + "/AbsEncoderDeg", Math.toDegrees(getAbsoluteEncoderRad()));

        // Show the turning motor encoder value in radians on the dashboard
        Logger.recordOutput("SwerveDrive/" + m_name + "/MotorTurningEncoderRad", getTurningEncoderPositionRad());

        // Show the turning motor encoder value in degrees on the dashboard
        Logger.recordOutput("SwerveDrive/" + m_name + "/MotorTurningEncoderDeg", Math.toDegrees(getTurningEncoderPositionRad()));

        // Show the target state speed and rotation in radians and degrees on the dashboard
        Logger.recordOutput("SwerveDrive/" + m_name + "/TargetSpeed", state.speedMetersPerSecond);
        Logger.recordOutput("SwerveDrive/" + m_name + "/TargetRotationRad", state.angle.getRadians());
        Logger.recordOutput("SwerveDrive/" + m_name + "/TargetRotationDeg", state.angle.getDegrees());

        // Log the state and position of the swerve module
        Logger.recordOutput("SwerveDrive/" + m_name + "/State", state);
        Logger.recordOutput("SwerveDrive/" + m_name + "/Position", getPosition());

        /*if (resetAngleTimer.get() > 0.5) {
            resetTurningMotorEncoder();
            resetAngleTimer.reset();
            resetAngleTimer.start();
        }*/
    }
}
