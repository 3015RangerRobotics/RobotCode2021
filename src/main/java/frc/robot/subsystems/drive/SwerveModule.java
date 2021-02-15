package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
    private TalonSRX rotationMotor;
    private double rotationOffset;
//    private ProfiledPIDController rotationPIDController;
//    private SimpleMotorFeedforward rotationMotorFeedforward;

    SwerveModule(int driveChannel, int rotationChannel, double rotationOffset) {
        this.rotationOffset = rotationOffset;
        driveMotor = new TalonFX(driveChannel);
        driveMotor.configFactoryDefault();
        rotationMotor = new TalonSRX(rotationChannel);
        rotationMotor.configFactoryDefault();

//        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
//        driveMotor.configSelectedFeedbackCoefficient(Constants.SWERVE_METERS_PER_PULSE);
        driveMotor.setInverted(false);
        driveMotor.setSensorPhase(false);
        driveMotor.config_kP(0, Constants.SWERVE_DRIVE_P);
        driveMotor.config_kI(0, Constants.SWERVE_DRIVE_I);
        driveMotor.config_kD(0, Constants.SWERVE_DRIVE_D);
        driveMotor.config_kF(0, 1023 / Constants.SWERVE_MAX_VELOCITY);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, 10);
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        rotationMotor.configSelectedFeedbackCoefficient(Constants.SWERVE_DEGREES_PER_PULSE, 0, 10);
        rotationMotor.configSelectedFeedbackCoefficient(Constants.SWERVE_DEGREES_PER_PULSE, 1, 10);
        rotationMotor.configVoltageCompSaturation(12.5);
        rotationMotor.enableVoltageCompensation(true);
        rotationMotor.configFeedbackNotContinuous(true, 10);
        rotationMotor.setInverted(true);
        rotationMotor.setSensorPhase(true);
        rotationMotor.config_kP(0, Constants.SWERVE_ROTATION_P);
        rotationMotor.config_kI(0, Constants.SWERVE_ROTATION_I);
        rotationMotor.config_kD(0, Constants.SWERVE_ROTATION_D);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10);
//        rotationMotor.configNominalOutputForward(0.15);
//        rotationMotor.configNominalOutputReverse(-0.15);
//        rotationMotor.configAllowableClosedloopError(0, 0.5);

//        rotationPIDController = new ProfiledPIDController(Constants.SWERVE_ROTATION_P, Constants.SWERVE_ROTATION_I, Constants.SWERVE_ROTATION_D,
//                new TrapezoidProfile.Constraints(Constants.SWERVE_MAX_ANGULAR_VELOCITY, Constants.SWERVE_MAX_ANGULAR_ACCEL));
//        rotationPIDController.enableContinuousInput(-180, 180);

//        rotationMotorFeedforward = new SimpleMotorFeedforward(0, Constants.SWERVE_ROTATION_KV, Constants.SWERVE_ROTATION_KA);
    }

    public void resetEncoders() {
//        rotationEncoder.reset();
        rotationMotor.setSelectedSensorPosition(0);
        driveMotor.setSelectedSensorPosition(0);
    }

    public void setRotationMotorVoltage(double voltage) {
//        rotationMotor.setVoltage(voltage);
    }

    public void setDriveMotor(ControlMode controlMode, double outputValue) {
        driveMotor.set(controlMode, outputValue);
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity() * 10, Rotation2d.fromDegrees(rotationMotor.getSelectedSensorPosition()));
    }

    public double getVelocity(){
        return driveMotor.getSelectedSensorVelocity() * 10 * Constants.SWERVE_METERS_PER_PULSE;
    }

    public void setRotationPosition(double degrees){
        double currentPosAbs = rotationMotor.getSelectedSensorPosition(1) - 180;
        double currentPosRel = rotationMotor.getSelectedSensorPosition(0);
        double delta = degrees - currentPosAbs - rotationOffset;
        if(delta > 180){
            delta -= 360;
        }else if(delta < -180){
            delta += 360;
        }
//        System.out.println(currentPosRel + delta);
        rotationMotor.set(ControlMode.Position, currentPosRel + delta);
//        rotationMotor.set(ControlMode.Position)
//        rotationMotor.set(ControlMode.PercentOutput, 0.5);
    }

    public void setSwerveModuleState(SwerveModuleState state) {
//        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(rotationEncoder.getDistance()));
//        double rotationPIDOutput = rotationPIDController.calculate(rotationEncoder.getDistance(), optimizedState.angle.getDegrees());
//        rotationPIDOutput += rotationMotorFeedforward.calculate(rotationPIDController.getSetpoint().velocity);
//        setDriveMotor(ControlMode.Velocity, optimizedState.speedMetersPerSecond);
//        setRotationMotorVoltage(rotationPIDOutput);
    }

}
