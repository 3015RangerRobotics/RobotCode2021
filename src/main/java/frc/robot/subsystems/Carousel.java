package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Carousel extends SubsystemBase {
	private VictorSP motor1;
	private VictorSP motor2;
	private VictorSP motor3;
	private VictorSP motor4;
	private VictorSP motor5;
	
	private DigitalInput switch1;
	private DigitalInput switch2;
	private DigitalInput switch3;
	private DigitalInput switch4;
	private DigitalInput switch5;
	
	private int ballCounter = 0;
	
	private static boolean isPaused = false;
	
	public enum State {
		kPurgeBall1,
		kPurgeBall2,
		kPurgeBall3,
		kPurgeBall4,
		kPurgeBall5,
		kShootBall1,
		kShootBall2,
		kShootBall3,
		kShootBall4,
		kShootBall5,
		kFillTo1,
		kFillTo2,
		kFillTo3,
		kFillTo4,
		kFillTo5,
		kOff
	}
	
	public State state = State.kOff;
	
	public Carousel() {
		motor1 = new VictorSP(Constants.CAROUSEL_MOTOR1); //Motor closest to shooter, used to push balls up to turret
		motor2 = new VictorSP(Constants.CAROUSEL_MOTOR2); //2nd motor closest to shooter
		motor3 = new VictorSP(Constants.CAROUSEL_MOTOR3); //3rd motor closest to shooter
		motor4 = new VictorSP(Constants.CAROUSEL_MOTOR4); //4th motor closest to shooter
		motor5 = new VictorSP(Constants.CAROUSEL_MOTOR5); //5th motor closest to shooter
		
		switch1 = new DigitalInput(Constants.CAROUSEL_BALL_SENSOR1); //assigned to motor 1
		switch2 = new DigitalInput(Constants.CAROUSEL_BALL_SENSOR2); //assigned to motor 2
		switch3 = new DigitalInput(Constants.CAROUSEL_BALL_SENSOR3); //assigned to motor 3
		switch4 = new DigitalInput(Constants.CAROUSEL_BALL_SENSOR4); //assigned to motor 4
		switch5 = new DigitalInput(Constants.CAROUSEL_BALL_SENSOR5); //assigned to motor 5
		
		motor1.setInverted(false);
		motor2.setInverted(true);
		motor3.setInverted(false);
		motor4.setInverted(true);
		motor5.setInverted(true);
	}
	
	@Override
	public void periodic() {
		SmartDashboard.putBoolean("switch1", isBall1Present());
		SmartDashboard.putBoolean("switch2", isBall2Present());
		SmartDashboard.putBoolean("switch3", isBall3Present());
		SmartDashboard.putBoolean("switch4", isBall4Present());
		SmartDashboard.putBoolean("switch5", isBall5Present());
		
		switch (state) {
			case kPurgeBall5:
				setMotors(0, 0, 0, 0, Constants.CAROUSEL_PURGE_SPEED5);
				if (!isBall5Present()) {
					state = State.kPurgeBall4;
				}
				break;
			
			case kPurgeBall4:
				setMotors(0, 0, 0, Constants.CAROUSEL_PURGE_SPEED4, Constants.CAROUSEL_PURGE_SPEED5);
				if (!isBall4Present()) {
					state = State.kPurgeBall3;
				}
				break;
			
			case kPurgeBall3:
				setMotors(0, 0, Constants.CAROUSEL_PURGE_SPEED3, Constants.CAROUSEL_PURGE_SPEED4,
						Constants.CAROUSEL_PURGE_SPEED5);
				if (!isBall3Present()) {
					state = State.kPurgeBall2;
				}
				break;
			
			case kPurgeBall2:
				setMotors(0, Constants.CAROUSEL_PURGE_SPEED2, Constants.CAROUSEL_PURGE_SPEED3,
						Constants.CAROUSEL_PURGE_SPEED4, Constants.CAROUSEL_PURGE_SPEED5);
				if (!isBall2Present()) {
					state = State.kPurgeBall1;
				}
				break;
			
			case kPurgeBall1:
				setMotors(Constants.CAROUSEL_PURGE_SPEED1, Constants.CAROUSEL_PURGE_SPEED2,
						Constants.CAROUSEL_PURGE_SPEED3, Constants.CAROUSEL_PURGE_SPEED4,
						Constants.CAROUSEL_PURGE_SPEED5);
				break;
			
			case kShootBall1:
				setMotors(Constants.CAROUSEL_SHOOT_SPEED, 0, 0, 0, 0);
				break;
			
			case kShootBall2:
				setMotors(Constants.CAROUSEL_SHOOT_SPEED, Constants.CAROUSEL_SHOOT_SPEED, 0, 0, 0);
				break;
			
			case kShootBall3:
				setMotors(Constants.CAROUSEL_SHOOT_SPEED, Constants.CAROUSEL_SHOOT_SPEED,
						Constants.CAROUSEL_SHOOT_SPEED, 0, 0);
				break;
			
			case kShootBall4:
				setMotors(Constants.CAROUSEL_SHOOT_SPEED, Constants.CAROUSEL_SHOOT_SPEED,
						Constants.CAROUSEL_SHOOT_SPEED, Constants.CAROUSEL_SHOOT_SPEED, 0);
				break;
			
			case kShootBall5:
				setMotors(Constants.CAROUSEL_SHOOT_SPEED, Constants.CAROUSEL_SHOOT_SPEED,
						Constants.CAROUSEL_SHOOT_SPEED, Constants.CAROUSEL_SHOOT_SPEED,
						Constants.CAROUSEL_SHOOT_SPEED);
				break;
			
			case kFillTo1:
				if (ballCounter >= 1){
					state = State.kFillTo2;
				}else {
					setMotors(Constants.CAROUSEL_IN_SPEED_ACTIVE, Constants.CAROUSEL_IN_SPEED_PASSIVE,
							Constants.CAROUSEL_IN_SPEED_PASSIVE, Constants.CAROUSEL_IN_SPEED_PASSIVE,
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
					setMotors(0, Constants.CAROUSEL_IN_SPEED_ACTIVE, Constants.CAROUSEL_IN_SPEED_PASSIVE,
							Constants.CAROUSEL_IN_SPEED_PASSIVE, Constants.CAROUSEL_IN_SPEED_PASSIVE);
					if (isBall2Present()) {
						ballCounter = 2;
						state = State.kFillTo3;
					}
					break;
				}
			
			case kFillTo3:
				if (ballCounter >= 3){
					state = State.kFillTo4;
				}else {
					setMotors(0, 0, Constants.CAROUSEL_IN_SPEED_ACTIVE, Constants.CAROUSEL_IN_SPEED_PASSIVE,
							Constants.CAROUSEL_IN_SPEED_PASSIVE);
					if (isBall3Present()) {
						ballCounter = 3;
						state = State.kFillTo4;
					}
					break;
				}
			
			case kFillTo4:
				if (ballCounter >= 4){
					state = State.kFillTo5;
				}else {
					setMotors(0, 0, 0, Constants.CAROUSEL_IN_SPEED_ACTIVE, Constants.CAROUSEL_IN_SPEED_PASSIVE);
					if (isBall4Present()) {
						ballCounter = 4;
						state = State.kFillTo5;
					}
					break;
				}
			
			case kFillTo5:
				if(ballCounter >= 5){
					state = State.kOff;
				}else {
					setMotors(0, 0, 0, 0, Constants.CAROUSEL_IN_SPEED_ACTIVE);
					if (isBall5Present()) {
						ballCounter = 5;
						state = State.kOff;
					}
					break;
				}
			
			
			case kOff:
			default:
				//Turns all motors off
				setMotors(0, 0, 0, 0, 0);
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
	 * @param speed4 Speed of motor 4
	 * @param speed5 Speed of motor 5
	 */
	private void setMotors(double speed1, double speed2, double speed3, double speed4, double speed5) {
		//set motors for percent ouput
		if (isPaused()) {
			motor1.set(0);
			motor2.set(0);
			motor3.set(0);
			motor4.set(0);
			motor5.set(0);
		} else {
			motor1.set(speed1);
			motor2.set(speed2);
			motor3.set(speed3);
			motor4.set(speed4);
			motor5.set(speed5);
		}
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
	
	/**
	 * Used to determine the state of the fourth switch closest to the shooter (a.k.a switch 4)
	 *
	 * @return state of switch, true if pressed.
	 */
	public boolean isBall4Present() {
		return !switch4.get();
	}
	
	/**
	 * Used to determine the state of the farthest switch from the shooter (a.k.a switch 5)
	 *
	 * @return state of switch, true if pressed.
	 */
	public boolean isBall5Present() {
		return !switch5.get();
	}
}
