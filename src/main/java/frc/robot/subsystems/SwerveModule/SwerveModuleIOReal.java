package frc.robot.subsystems.SwerveModule;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class SwerveModuleIOReal implements SwerveModuleIO {
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
     * Current state of the swerve module
     */
    private SwerveModuleState state = new SwerveModuleState();

    /**
     * Linear filter for updating the turning motor encoder constantly with the absolute value (It doesn't update constantly so we need to smooth it)
     */
    // private LinearFilter turnEncoderSyncFilter = LinearFilter.singlePoleIIR(1, 0.2);

    /**
     * Turning motor PID controller
     */
    private SparkPIDController m_driveMotorPIDController;

    /**
     * Last timestamp when the motor encoders updated
     */
    private double lastMotorEncoderUpdateTime = Timer.getFPGATimestamp();

    /**
     * Swerve module constructor
     * @param Arr Configuration array
     */
    public SwerveModuleIOReal(Object[] Arr) {
        // Get variables from options and add them to the class
        this.m_turningEncoderOffsetRad = (double)Arr[0]; 
        this.m_turningAbsoluteEncoderInverted = (boolean)Arr[1]; 
        this.m_turningAbsoluteEncoder = new CANcoder((int)Arr[2]);

        this.m_driveMotor = new CANSparkMax((int)Arr[3], MotorType.kBrushless); 
        this.m_turningMotor = new CANSparkMax((int)Arr[4], MotorType.kBrushless);

        this.m_driveMotor.setInverted((boolean)Arr[5]); 
        this.m_turningMotor.setInverted((boolean)Arr[6]); 

        this.m_driveMotorEncoder = m_driveMotor.getEncoder();
        this.m_turningMotorEncoder = m_turningMotor.getEncoder();

        this.m_turningAbsoluteEncoder.getConfigurator().apply(
            new MagnetSensorConfigs()
            .withMagnetOffset((m_turningEncoderOffsetRad + Constants.Swerve.Module.kGlobalTurningOffsetRad) / (2 * Math.PI))
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        );

        this.m_driveMotorPIDController = m_driveMotor.getPIDController();

        // Configure encoder conversions
        this.m_driveMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kDriveEncoder_RotationToMeter); 
        this.m_driveMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kDriveEncoder_RPMToMeterPerSecond);
        this.m_turningMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kTurningEncoder_RotationToRadian); 
        this.m_turningMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kTurningEncoder_RPMToRadianPerSecond);

        // PID Continuous input
        Constants.Swerve.Module.getTurningPIDController().enableContinuousInput(0, Math.PI * 2);

        // Set module name
        m_name = (String) Arr[7];

        // Reset the motor encoder with the value of the absolute encoder + the offset
        resetMotorEncoders();
    }

    /**
     * Reset the motor encoders
     * This function is used to reset the motor encoders to the absolute encoder value when initializing
     */
    private void resetMotorEncoders() {
        // Make sure it's using radians instead of rotations
        this.m_turningMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kTurningEncoder_RotationToRadian); 
        this.m_turningMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kTurningEncoder_RPMToRadianPerSecond);

        // Reset the motor encoder with the value of the absolute encoder
        // The getAbsoluteEncoderRad() function returns the value in radians
        m_turningMotorEncoder.setPosition(getAbsoluteEncoderRad());

        // We reset the drive motor position to start measuring distance from 0
        m_driveMotorEncoder.setPosition(0);
    }

    // Return the absolute encoder position in radians
    public double getAbsoluteEncoderRad() {
        return ((m_turningAbsoluteEncoder.getPosition().refresh().getValue() * 2 * Math.PI) % (2 * Math.PI)) * (m_turningAbsoluteEncoderInverted ? -1.0 : 1.0);
    }

    /**
     * Get the turning motor encoder position in radians
     * @return double
     */
    public double getTurningEncoderPositionRad() {
        // Make sure it's using radians instead of rotations
        this.m_turningMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kTurningEncoder_RotationToRadian); 
        this.m_turningMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kTurningEncoder_RPMToRadianPerSecond);
        
        // Return the motor encoder position in radians
        return this.m_turningMotorEncoder.getPosition() % (2 * Math.PI);
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
        // Check if the speed or rotation difference is meaningful
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
    
        // Set the state
        this.state = state;

        // Optimize the state
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, new Rotation2d(getTurningEncoderPositionRad()));
    
        // ! PAST IMPLEMENTATION (NOT ACCURATE)
        // Set the drive motor speed
        // The speed is given in meters per second, so we need to convert from [-1, 1]
        // To do that we divide it by the max speed of robot
        // m_driveMotor.set(optimizedState.speedMetersPerSecond / Constants.Swerve.Physical.kMaxSpeedMetersPerSecond);
        m_driveMotorPIDController.setP(0.175);
        m_driveMotorPIDController.setI(0.0000000005);
        m_driveMotorPIDController.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
        
        // Show the applied speed percentage on the dashboard
        Logger.recordOutput("SwerveDrive/" + m_name + "/AppliedSpeedPercentage", m_driveMotor.getAppliedOutput());
        Logger.recordOutput("SwerveDrive/" + m_name + "/TargetSpeedPMpS", optimizedState.speedMetersPerSecond);
        Logger.recordOutput("SwerveDrive/" + m_name + "/RealSpeedPMpS", m_driveMotorEncoder.getVelocity());
    
        // Set the turning motor speed
        // The speed is not given to us, rather the angle we want to turn to
        // So we need to calculate the difference between the current angle and the target angle
        // Then we use the PID controller to calculate the speed we need to turn at
        m_turningMotor.set(Constants.Swerve.Module.getTurningPIDController().calculate(getTurningEncoderPositionRad(), optimizedState.angle.getRadians()));
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
     * Get the position of the swerve module (Displacement and rotation)
     * @return SwerveModulePosition
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveMotorEncoder.getPosition(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    /**
     * Sync the turn motor encoder with the absolute encoder constantly smoothly with a linear filter
     */
    public void syncTurnMotorEncoderSmooth() {
        // Make sure it's using radians instead of rotations
        this.m_turningMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kTurningEncoder_RotationToRadian);
        
        // Reset the motor encoder with the value of the absolute encoder
        // The getAbsoluteEncoderRad() function returns the value in radians
        m_turningMotorEncoder.setPosition(getAbsoluteEncoderRad());
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Set the angle and speed of the swerve module
        inputs.angle = getAbsoluteEncoderRad();
        inputs.speed = m_driveMotorEncoder.getVelocity();
        inputs.distance = m_driveMotorEncoder.getPosition();
    }

    public void periodic() {
        // Current FPGA time
        double currentTime = Timer.getFPGATimestamp();

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

        // Sync the turn motor encoder with the absolute encoder constantly
        if (currentTime - lastMotorEncoderUpdateTime >= 0.5) {
            syncTurnMotorEncoderSmooth();
            lastMotorEncoderUpdateTime = currentTime;
            System.out.println(getName() + " Swerve Module Real Rotation Updated");
        }
    }
}
