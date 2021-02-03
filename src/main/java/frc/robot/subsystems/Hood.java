// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Hood extends SubsystemBase {
    private CANSparkMax hoodMotor;
    private double setPos = 0;

    private enum State {
        kSetPosition,
        kAutoPosition,
        kNeutral,
        kHoming
    }

    public State state = State.kNeutral;

    public Hood() {
       hoodMotor = new CANSparkMax(Constants.HOOD_MOTOR_CHANNEL, MotorType.kBrushless);
       hoodMotor.restoreFactoryDefaults();
       hoodMotor.getEncoder().setPositionConversionFactor(Constants.HOOD_DEGREES_PER_ROTATION);
       hoodMotor.getPIDController().setP(Constants.HOOD_CONTROLLER_P);
       hoodMotor.getPIDController().setI(Constants.HOOD_CONTROLLER_I);
       hoodMotor.getPIDController().setD(Constants.HOOD_CONTROLLER_D);
       hoodMotor.getPIDController().setOutputRange(-1, 1);
       hoodMotor.setInverted(false);
       hoodMotor.getEncoder().setInverted(false);
       hoodMotor.setSoftLimit(SoftLimitDirection.kForward, 90);
       hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
       hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
       enableReverseSoftLimit(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (getReverseLimit()){
            hoodMotor.getEncoder().setPosition(0);
        }

        switch (state){
            case kSetPosition:
                setHoodPosition(setPos);
                break;
            case kAutoPosition:
                setPos = getAutoPosition();
                setHoodPosition(setPos);
                break;
            case kHoming:
                setHoodOutputPercentage(-0.2);
                if(getReverseLimit()){
                    setStateNeutral();
                    enableReverseSoftLimit(true);
                }
                break;
            case kNeutral:
            default:
                setHoodOutputPercentage(0);
                break;
        }
    }

    public void setStatePosition(double pos){
        setPos = pos;
        state = State.kSetPosition;
    }

    public void setStateAutoPosition(){
        state = State.kAutoPosition;
    }

    public void setStateHoming(){
        state = State.kHoming;
    }

    public void setStateNeutral(){
        state = State.kNeutral;
    }

    public double getHoodPosition() {
      return hoodMotor.getEncoder().getPosition();
      }

    public void setHoodPosition(double position) {
        hoodMotor.getPIDController().setReference(position, ControlType.kPosition);
    }

    public double getAutoPosition() {
        double d = RobotContainer.limelight.getRobotToTargetDistance();
        return Constants.HOOD_AUTO_POSITION_TABLE.lookup(d);
    }

    public boolean getReverseLimit() {
        return hoodMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
    }

    public void setHoodOutputPercentage(double percentage){
        hoodMotor.set(percentage);
    }

    public void enableReverseSoftLimit(boolean enabled){
        hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
    }
}