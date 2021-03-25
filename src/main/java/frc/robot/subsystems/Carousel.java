package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Carousel extends SubsystemBase {
	private PWMVictorSPX motor1;
	private PWMVictorSPX motor2;
	private PWMVictorSPX motor3;
	private VictorSP intake;

	private DigitalInput switch1;
	private DigitalInput switch2;
	private DigitalInput switch3;
	
	private int ballCounter = 0;
	
	private static boolean isPaused = false;

	private PIDController motor1PidController;

	private Encoder motor1Encoder;

	private double setVelocity;
	
	public enum State {
		kPurgeBall1,
		kPurgeBall2,
		kPurgeBall3,
		kShootBall1,
		kShootBall2,
		kShootBall3,
		kFillTo1,
		kFillTo2,
		kFillTo3,
		kOff
	}
	
	public State state = State.kOff;
	
	public Carousel() {
		motor1 = new PWMVictorSPX(Constants.CAROUSEL_MOTOR1); //Motor closest to shooter, used to push balls up to turret
		motor2 = new PWMVictorSPX(Constants.CAROUSEL_MOTOR2); //2nd motor closest to shooter
		motor3 = new PWMVictorSPX(Constants.CAROUSEL_MOTOR3); //3rd motor closest to shooter
		intake = new VictorSP(Constants.CAROUSEL_INTAKE_MOTOR);
		
		switch1 = new DigitalInput(Constants.CAROUSEL_BALL_SENSOR1); //assigned to motor 1
		switch2 = new DigitalInput(Constants.CAROUSEL_BALL_SENSOR2); //assigned to motor 2
		switch3 = new DigitalInput(Constants.CAROUSEL_BALL_SENSOR3); //assigned to motor 3
		
		motor1.setInverted(false);
		motor2.setInverted(false);
		motor3.setInverted(true);
		intake.setInverted(false);

		motor1PidController = new PIDController(Constants.CAROUSEL_SHOOT_P, Constants.CAROUSEL_SHOOT_I, Constants.CAROUSEL_SHOOT_D);
		motor1Encoder = new Encoder(Constants.CAROUSEL_ENCODER1, Constants.CAROUSEL_ENCODER2, Constants.CAROUSEL_ENCODER3);
//		motor1Encoder.setDistancePerPulse(1.0 / Constants.CAROUSEL_PULSES_PER_ROTATION);

		motor1Encoder.setReverseDirection(true);
	}
	
	@Override
	public void periodic() {
		SmartDashboard.putBoolean("switch1", isBall1Present());
		SmartDashboard.putBoolean("switch2", isBall2Present());
		SmartDashboard.putBoolean("switch3", isBall3Present());
		SmartDashboard.putNumber("Intake Motor1 Velocity", getMotor1Velocity());
//		SmartDashboard.putNumber("PIDTarget", setVelocity);
//		SmartDashboard.putNumber("PIDActual", getMotor1Velocity());

		switch (state) {
			case kPurgeBall3:
				setMotors(0, 0, Constants.CAROUSEL_PURGE_SPEED3);
				if (!isBall3Present()) {
					state = State.kPurgeBall2;
				}
				break;
			
			case kPurgeBall2:
				setMotors(0, Constants.CAROUSEL_PURGE_SPEED2, Constants.CAROUSEL_PURGE_SPEED3);
				if (!isBall2Present()) {
					state = State.kPurgeBall1;
				}
				break;
			
			case kPurgeBall1:
				setMotors(Constants.CAROUSEL_PURGE_SPEED1, Constants.CAROUSEL_PURGE_SPEED2,
						Constants.CAROUSEL_PURGE_SPEED3);
				break;
			
			case kShootBall1:
				setMotorsVelocity(Constants.CAROUSEL_SHOOT_PERCENTAGE, 0, 0);
				break;
			
			case kShootBall2:
				setMotorsVelocity(Constants.CAROUSEL_SHOOT_PERCENTAGE, Constants.CAROUSEL_SHOOT_PERCENTAGE, 0);
				break;
			
			case kShootBall3:
				setMotorsVelocity(Constants.CAROUSEL_SHOOT_PERCENTAGE, Constants.CAROUSEL_SHOOT_PERCENTAGE,
						Constants.CAROUSEL_SHOOT_PERCENTAGE);
				break;

			case kFillTo1:
				if (ballCounter >= 1){
					state = State.kFillTo2;
				}else {
					setMotors(Constants.CAROUSEL_IN_SPEED_ACTIVE, Constants.CAROUSEL_IN_SPEED_PASSIVE,
							Constants.CAROUSEL_IN_SPEED_PASSIVE);
					if (isBall1Present()) {
						ballCounter = 1;
						state = State.kFillTo2;
					}
					break;
				}
			
			case kFillTo2:
				if (ballCounter >= 2){
					state = State.kFillTo3;
				}else {
					setMotors(0, Constants.CAROUSEL_IN_SPEED_ACTIVE, Constants.CAROUSEL_IN_SPEED_PASSIVE);
					if (isBall2Present()) {
						ballCounter = 2;
						state = State.kFillTo3;
					}
					break;
				}
			
			case kFillTo3:
				if (ballCounter >= 3){
					state = State.kOff;
				}else {
					setMotors(0, 0, Constants.CAROUSEL_IN_SPEED_ACTIVE);
					if (isBall3Present()) {
						ballCounter = 3;
						state = State.kOff;
					}
					break;
				}
			
			
			case kOff:
			default:
				//Turns all motors off
				setMotors(0, 0, 0);
		}
	}
	
	/**
	 * Sets the state of the motor based on desired operation
	 *
	 * @param newState The new State you would like to change to
	 */
	public void setState(State newState) {
		this.state = newState;
	}
	
	public void setBallCounter(int ballCounter){
		this.ballCounter = ballCounter;
	}
	
	/**
	 * gets the current state of the enumerator
	 *
	 * @return state of enumerator
	 */
	public State getState() {
		return state;
	}
	
	/**
	 * Get if the ball handler is in a paused state
	 * @return is the ball handler paused
	 */
	public boolean isPaused() {
		return isPaused;
	}
	
	/**
	 * Set the paused state
	 * @param pausedState Should the ball handler be paused
	 */
	public void setPaused(boolean pausedState) {
		isPaused = pausedState;
	}
	
	/**
	 * Sets all five motors to a specific speed.
	 *
	 * @param speed1 Speed of motor 1
	 * @param speed2 Speed of motor 2
	 * @param speed3 Speed of motor 3
	 */
	private void setMotors(double speed1, double speed2, double speed3) {
		//set motors for percent ouput
		setVelocity = 0;
		if (isPaused()) {
			motor1.set(0);
			motor2.set(0);
			motor3.set(0);
			intake.set(0);
		} else {
			motor1.set(speed1);
			motor2.set(speed2);
			motor3.set(speed3);
			intake.set(Math.signum(speed3));
		}
	}

	public void setMotorsVelocity(double speed1vel, double speed2pct, double speed3pct) {
		if (isPaused()) {
			setVelocity = 0;
			motor1.set(0);
			motor2.set(0);
			motor3.set(0);
		} else {
			setVelocity = speed1vel;
			motor1.set(speed1vel);
			motor2.set(speed2pct);
			motor3.set(speed3pct);
		}
	}

	public double getMotor1Velocity() {
		return motor1Encoder.getRate();
	}

	public double getMotor1Position() {
		return motor1Encoder.get();
	}
	
	/**
	 * Used to determine the state of the switch closest to the shooter (a.k.a switch 1)
	 *
	 * @return state of switch, true if pressed.
	 */
	public boolean isBall1Present() {
		return !switch1.get();
	}
	
	/**
	 * Used to determine the state of the second switch closest to the shooter (a.k.a switch 2)
	 *
	 * @return state of switch, true if pressed.
	 */
	public boolean isBall2Present() {
		return !switch2.get();
	}
	
	/**
	 * Used to determine the state of the third switch closest to the shooter (a.k.a switch 3)
	 *
	 * @return state of switch, true if pressed.
	 */
	public boolean isBall3Present() {
		return !switch3.get();
	}

}
