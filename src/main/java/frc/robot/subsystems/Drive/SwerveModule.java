package frc.robot.subsystems.Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {
    private TalonFX driveMotor;
    private VictorSP rotationMotor;
    private Encoder rotationEncoder;
    private ProfiledPIDController rotationPIDController;
    private SimpleMotorFeedforward rotationMotorFeedforward;

    SwerveModule(int driveChannel, int rotationChannel, int rotationEncoder0, int rotationEncoder1) {
        driveMotor = new TalonFX(driveChannel);
        driveMotor.configFactoryDefault();
        rotationMotor = new VictorSP(rotationChannel);
        rotationEncoder = new Encoder(rotationEncoder0, rotationEncoder1);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        driveMotor.configSelectedFeedbackCoefficient(Constants.SWERVE_METERS_PER_PULSE);
        driveMotor.setInverted(false);
        driveMotor.setSensorPhase(false);
        driveMotor.config_kP(0, Constants.SWERVE_DRIVE_P);
        driveMotor.config_kI(0, Constants.SWERVE_DRIVE_I);
        driveMotor.config_kD(0, Constants.SWERVE_DRIVE_D);
        driveMotor.config_kF(0, 1023 / Constants.SWERVE_MAX_VELOCITY);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        rotationMotor.setInverted(false);

        rotationEncoder.setDistancePerPulse(Constants.SWERVE_DEGREES_PER_PULSE);
        rotationEncoder.setReverseDirection(false);

        rotationPIDController = new ProfiledPIDController(Constants.SWERVE_ROTATION_P, Constants.SWERVE_ROTATION_I, Constants.SWERVE_ROTATION_D,
                new TrapezoidProfile.Constraints(Constants.SWERVE_MAX_ANGULAR_VELOCITY, Constants.SWERVE_MAX_ANGULAR_ACCEL));
        rotationPIDController.enableContinuousInput(-180, 180);

        rotationMotorFeedforward = new SimpleMotorFeedforward(0, Constants.SWERVE_ROTATION_KV, Constants.SWERVE_ROTATION_KA);
    }

    public void resetEncoders() {
        rotationEncoder.reset();
        driveMotor.setSelectedSensorPosition(0);
    }

    public void setRotationMotorVoltage(double voltage) {
        rotationMotor.setVoltage(voltage);
    }

    public void setDriveMotor(ControlMode controlMode, double outputValue) {
        driveMotor.set(controlMode, outputValue);
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity() * 10, Rotation2d.fromDegrees(rotationEncoder.getDistance()));
    }

    public void setSwerveModuleState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(rotationEncoder.getDistance()));
        double rotationPIDOutput = rotationPIDController.calculate(rotationEncoder.getDistance(), optimizedState.angle.getDegrees());
        rotationPIDOutput += rotationMotorFeedforward.calculate(rotationPIDController.getSetpoint().velocity);
        setDriveMotor(ControlMode.Velocity, optimizedState.speedMetersPerSecond);
        setRotationMotorVoltage(rotationPIDOutput);
    }

}
