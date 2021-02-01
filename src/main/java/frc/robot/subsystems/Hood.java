// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
    private CANSparkMax hoodMotor;

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
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (getReverseLimit()){
            hoodMotor.getEncoder().setPosition(0);
        }
    }

    public double getHoodPosition() {
      return hoodMotor.getEncoder().getPosition();
      }

    public void setHoodPosition(double position) {
        hoodMotor.getPIDController().setReference(position, ControlType.kPosition);
    }

    public boolean getReverseLimit() {
        return hoodMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
    }

    public void setHoodOutputPercentage(double percentage){
        hoodMotor.set(percentage);
    }
}