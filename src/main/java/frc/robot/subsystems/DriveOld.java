package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveOld extends SubsystemBase {
    public TalonFX rightMaster;
    private TalonFX rightFollower;

    private TalonFX leftMaster;
    private TalonFX leftFollower;

    public DriveOld() {
        this.rightMaster = new TalonFX(1);
        this.rightFollower = new TalonFX(2);
        this.leftMaster = new TalonFX(3);
        this.leftFollower = new TalonFX(4);

        rightFollower.follow(rightMaster);
        leftFollower.follow(leftMaster);

        rightMaster.setInverted(true);
        rightFollower.setInverted(true);
        leftMaster.setInverted(false);
        leftFollower.setInverted(false);

        rightMaster.enableVoltageCompensation(true);
        rightMaster.configVoltageCompSaturation(12.5);
        leftMaster.enableVoltageCompensation(true);
        leftMaster.configVoltageCompSaturation(12.5);

        rightMaster.setSensorPhase(true);
        leftMaster.setSensorPhase(false);

        resetEncoders();
        enableBrakeMode();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("right", rightMaster.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("left", leftMaster.getSelectedSensorPosition(0));
    }

    /**
     * Reset the drive encoders
     */
    public void resetEncoders() {
        leftMaster.getSensorCollection().setIntegratedSensorPosition(0, 20);
        rightMaster.getSensorCollection().setIntegratedSensorPosition(0, 20);
    }

    /**
     * Enable coast mode
     */
    public void enableCoastMode(){
        rightMaster.setNeutralMode(NeutralMode.Coast);
        rightFollower.setNeutralMode(NeutralMode.Coast);
        leftMaster.setNeutralMode(NeutralMode.Coast);
        leftFollower.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Enable brake mode
     */
    public void enableBrakeMode(){
        rightMaster.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);
        leftMaster.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Set the output to the drive motors
     * @param mode The control mode to use
     * @param leftMotor Output value of the left motor
     * @param rightMotor Output value of the right motor
     */
    public void setMotorOutputs(ControlMode mode, double leftMotor, double rightMotor) {
        this.rightMaster.set(mode, rightMotor);
        this.leftMaster.set(mode, leftMotor);
    }

    /**
     * Set the motors using arcade drive
     * @param driveValue Forward/Reverse value
     * @param turnValue Turn value
     * @param squaredInputs Should the inputs be squared (increases control at low speeds)
     */
    public void arcadeDrive(double driveValue, double turnValue, boolean squaredInputs) {
        DriveSignal ds = DriveHelper.arcadeDrive(driveValue, turnValue, squaredInputs);
        setMotorOutputs(ControlMode.PercentOutput, ds.leftSignal, ds.rightSignal);
    }

    public void curvatureDrive(double throttle, double turn, boolean isQuickTurn, boolean squaredInputs) {
        DriveSignal ds = DriveHelper.curvatureDrive(throttle, turn, isQuickTurn, squaredInputs);
        setMotorOutputs(ControlMode.PercentOutput, ds.leftSignal, ds.rightSignal);
    }

    private static class DriveSignal {
        public double leftSignal;
        public double rightSignal;

        public DriveSignal(double leftSignal, double rightSignal) {
            this.leftSignal = leftSignal;
            this.rightSignal = rightSignal;
        }
    }

    private static class DriveHelper {
        private static final double kDeadband = 0.05;
        private static double quickStopAccumulator = 0.0;
        private static final double kTurnSensitivity = 1.0;

        /**
         * Tank drive helper
         *
         * @param left Left speed
         * @param right Right speed
         * @return Outputs for left and right motors
         */
        public static DriveSignal tankDrive(double left, double right) {
            return new DriveSignal(left, right);
        }

        /**
         * Arcade Drive
         *
         * @param moveValue Forward/Reverse speed
         * @param rotateValue Turn Speed
         * @param squaredInputs Should the inputs be squared (Increase control at low speeds)
         * @return Outputs for left and right motors
         */
        public static DriveSignal arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
            double leftMotorSpeed;
            double rightMotorSpeed;
            moveValue = handleDeadzone(moveValue, kDeadband);
            rotateValue = -handleDeadzone(rotateValue, kDeadband);
            if (squaredInputs) {
                if (moveValue >= 0.0) {
                    moveValue = moveValue * moveValue;
                } else {
                    moveValue = -(moveValue * moveValue);
                }
                if (rotateValue >= 0.0) {
                    rotateValue = rotateValue * rotateValue;
                } else {
                    rotateValue = -(rotateValue * rotateValue);
                }
            }
            if (moveValue > 0.0) {
                if (rotateValue > 0.0) {
                    leftMotorSpeed = moveValue - rotateValue;
                    rightMotorSpeed = Math.max(moveValue, rotateValue);
                } else {
                    leftMotorSpeed = Math.max(moveValue, -rotateValue);
                    rightMotorSpeed = moveValue + rotateValue;
                }
            } else {
                if (rotateValue > 0.0) {
                    leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                    rightMotorSpeed = moveValue + rotateValue;
                } else {
                    leftMotorSpeed = moveValue - rotateValue;
                    rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
                }
            }
            return new DriveSignal(leftMotorSpeed, rightMotorSpeed);
        }
        public static DriveSignal curvatureDrive(double throttle, double turn, boolean isQuickTurn, boolean squaredInputs) {
            throttle = handleDeadzone(throttle, kDeadband);
            turn = handleDeadzone(turn, kDeadband);

            double overPower;
            double angularPower;

            if (squaredInputs) {
                // square the inputs (while preserving the sign) to increase fine control
                // while permitting full power
                if (throttle >= 0.0) {
                    throttle = throttle * throttle;
                } else {
                    throttle = -(throttle * throttle);
                }
            }

            if (isQuickTurn) {
                if (Math.abs(throttle) < 0.2) {
                    double alpha = 0.1;
                    quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * limit(turn, 1.0) * 2;
                }
                overPower = 1.0;
                angularPower = turn;
            } else {
                overPower = 0.0;
                angularPower = Math.abs(throttle) * turn * kTurnSensitivity - quickStopAccumulator;
                if (quickStopAccumulator > 1) {
                    quickStopAccumulator -= 1;
                } else if (quickStopAccumulator < -1) {
                    quickStopAccumulator += 1;
                } else {
                    quickStopAccumulator = 0.0;
                }
            }

            double rightPwm = throttle - angularPower;
            double leftPwm = throttle + angularPower;
            if (leftPwm > 1.0) {
                rightPwm -= overPower * (leftPwm - 1.0);
                leftPwm = 1.0;
            } else if (rightPwm > 1.0) {
                leftPwm -= overPower * (rightPwm - 1.0);
                rightPwm = 1.0;
            } else if (leftPwm < -1.0) {
                rightPwm += overPower * (-1.0 - leftPwm);
                leftPwm = -1.0;
            } else if (rightPwm < -1.0) {
                leftPwm += overPower * (-1.0 - rightPwm);
                rightPwm = -1.0;
            }

            return new DriveSignal(leftPwm, rightPwm);
        }

        /**
         * Handles a deadzone
         *
         * @param value    The value to handle
         * @param deadzone The deadzone
         * @return The handled value
         */
        protected static double handleDeadzone(double value, double deadzone) {
            return (Math.abs(value) > Math.abs(deadzone)) ? limit(value, 1.0) : 0.0;
        }

        /**
         * Limits a number between a given range
         *
         * @param value    The value to limit
         * @param max The maximum value
         * @return The handled value
         */
        protected static double limit(double value, double max) {
            if (value > max) {
                return max;
            }
            if (value < -max) {
                return -max;
            }
            return value;
        }
    }
}