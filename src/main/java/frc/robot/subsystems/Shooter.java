package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
	private TalonFX shooter;
	private TalonFX shooterFollower;
	private double setSpeed;
	
	private enum State {
		kSetSpeed,
		kAutoSpeed,
		kAutoSpeedFender,
		kOff,
		kTesting
	}
	
	public State state = State.kOff;
	
	public Shooter() {
		shooter = new TalonFX(Constants.SHOOTER_MOTOR);
		shooterFollower = new TalonFX(Constants.SHOOTER_MOTOR_2);
		shooterFollower.setInverted(true);
		shooterFollower.follow(shooter);

		shooter.configFactoryDefault();
		
		shooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
		shooter.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10);
		
		shooter.setNeutralMode(NeutralMode.Coast);
		
		shooter.enableVoltageCompensation(true);
		shooter.configVoltageCompSaturation(12.5);
		
		shooter.setInverted(false);
		shooter.setSelectedSensorPosition(0);
		shooter.setSensorPhase(false);
		
		shooter.config_kP(0, Constants.SHOOTER_P);
		shooter.config_kI(0, Constants.SHOOTER_I);
		shooter.config_kD(0, Constants.SHOOTER_D);
		shooter.config_kF(0, Constants.SHOOTER_F);
		
		shooter.config_kP(1, Constants.SHOOTER_SHOOT_P);
		shooter.config_kI(1, Constants.SHOOTER_SHOOT_I);
		shooter.config_kD(1, Constants.SHOOTER_SHOOT_D);
		shooter.config_kF(1, Constants.SHOOTER_SHOOT_F);
		
	}
	
	@Override
	public void periodic() {
		SmartDashboard.putNumber("shooter speed", setSpeed);
//		SmartDashboard.putNumber("PIDTarget", setSpeed);
//		SmartDashboard.putNumber("PIDActual", getRPM());
		
		if(getRPM() >= 8500){
			state = State.kOff;
		}
		
		/*
		TODO
		This is all going to have to be redone if the shooter is not going to be on a turret this year.
		For now I'll copy the code for the turret over so this doesn't have errors, but we can remove it later
		if need be.
		 */
//		double turretPos = RobotContainer.turret.getPosition() + RobotContainer.limelight.getTargetAngleX();
		double turretOffset = 0;//2 * Math.abs(turretPos);
		
		switch(state) {
			case kSetSpeed:
				set(ControlMode.Velocity, (setSpeed + turretOffset) * Constants.SHOOTER_PULSES_PER_ROTATION / 600);
				break;
			
			case kAutoSpeed:
				setSpeed = getAutoSpeed(false);
				set(ControlMode.Velocity, (setSpeed + turretOffset) * Constants.SHOOTER_PULSES_PER_ROTATION / 600);
				break;
			
			case kAutoSpeedFender:
				if(!RobotContainer.limelight.hasTarget()){
//					RobotContainer.setDriverRumbleLeft(1);
//					RobotContainer.setDriverRumbleRight(1);
//					RobotContainer.setCoDriverRumbleLeft(1);
//					RobotContainer.setCoDriverRumbleRight(1);
				}else{
//					RobotContainer.setDriverRumbleLeft(0);
//					RobotContainer.setDriverRumbleRight(0);
//					RobotContainer.setCoDriverRumbleLeft(0);
//					RobotContainer.setCoDriverRumbleRight(0);
				}
				setSpeed = getAutoSpeed(true);
				set(ControlMode.Velocity, setSpeed * Constants.SHOOTER_PULSES_PER_ROTATION / 600);
				break;
			
			case kTesting:
				break;
			
			case kOff:
			default:
				setSpeed = 0;
				set(ControlMode.PercentOutput, 0);
				break;
		}
	}
	
	/**
	 * @return The RPM of the shooter wheel
	 */
	public double getRPM() {
		return (shooter.getSelectedSensorVelocity() * 10 * 60 / Constants.SHOOTER_PULSES_PER_ROTATION);
	}
	
	/**
	 * Set the output of the shooter wheel
	 * @param mode The control mode to use
	 * @param value The value to set
	 */
	public void set(ControlMode mode, double value) {
		shooter.set(mode, value);
	}
	
	/**
	 * Select which pid profile to use
	 * @param id The ID of the profile (0 - Prime PID, 1 - Shooting PID)
	 */
	public void selectProfileSlot(int id){
		shooter.selectProfileSlot(id, 0);
	}
	
	/**
	 * Is the shooter currently running
	 * @return True if the shooter is running
	 */
	public boolean isRunning() {
		return shooter.getMotorOutputPercent() > 0;
	}
	
	/**
	 * Put the shooter in a set speed state
	 * @param speed The speed in RPM to go to
	 */
	public void setStateSpeed(double speed) {
		setSpeed = speed;
		state = State.kSetSpeed;
	}
	
	/**
	 * Put the shooter in a testing state
	 */
	public void setStateTesting() {
		state = State.kTesting;
		set(ControlMode.PercentOutput, 0);
	}
	
	/**
	 * Put the shooter in the auto speed state
	 */
	public void setStateAutoSpeed() {
		state = State.kAutoSpeed;
	}
	
	/**
	 * Put the shooter in the auto speed state for the fender shot
	 */
	public void setStateAutoSpeedFender() {
		state = State.kAutoSpeedFender;
	}
	
	/**
	 * Put the shooter in the off state
	 */
	public void setStateOff() {
		state = State.kOff;
	}
	
	/**
	 * Get the auto speed
	 * @param isFender Is the speed for fender shot
	 * @return The calculated speed for the shooter in RPM
	 */
	public double getAutoSpeed(boolean isFender) {
		if(RobotContainer.limelight.hasTarget()) {
			double d = RobotContainer.limelight.getRobotToTargetDistance();
			if(!isFender) {
//           double rpm = 7430.1186 + (-255.07933*d) + (7.2472131*d*d); //Perfect ball
				return Constants.SHOOTER_AUTO_SPEED_TABLE.lookup(d);
			}else{
				
				if(d <= 8){
					return 7000;
				}else if(d <= 10){
					return 5000;
				}else if(d <= 20){
					return 3300;
				}else if(d <= 25){
					return 3100;
				}
				return 3300;
			}
		} else {
			return isFender ? 3300 : 5400;
		}
	}
	
	/**
	 * Is the shooter primed and ready to fire
	 * @return True if the shooter is primed
	 */
	public boolean isPrimed() {
		double turretPos = 0;//RobotContainer.turret.getPosition() + RobotContainer.limelight.getTargetAngleX();
		return Math.abs(setSpeed + (2 * Math.abs(turretPos)) - getRPM()) <= Constants.SHOOTER_TOLERANCE;
	}
}

