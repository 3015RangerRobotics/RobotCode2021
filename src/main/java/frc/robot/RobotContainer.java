// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static Drive drive;
    public static Hood hood;
    public static Carousel carousel;
    public static Limelight limelight;
    public static Shooter shooter;

    public static PowerDistributionPanel pdp = new PowerDistributionPanel(0);

    private static final XboxController driver = new XboxController(0);
    private static final XboxController coDriver = new XboxController(1);

    private static final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private static final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    public static final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    private static final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    private static final JoystickButton driverLB = new JoystickButton(driver, XboxController.Button.kBumperLeft.value);
    private static final JoystickButton driverRB = new JoystickButton(driver, XboxController.Button.kBumperRight.value);
    private static final DPadButton driverDUp = new DPadButton(driver, DPadButton.Value.kDPadUp);
    private static final DPadButton driverDDown = new DPadButton(driver, DPadButton.Value.kDPadDown);
    private static final DPadButton driverDLeft = new DPadButton(driver, DPadButton.Value.kDPadLeft);
    private static final DPadButton driverDRight = new DPadButton(driver, DPadButton.Value.kDPadRight);
    private static final TriggerButton driverLT = new TriggerButton(driver, Hand.kLeft);
    private static final TriggerButton driverRT = new TriggerButton(driver, Hand.kRight);
    private static final JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    private static final JoystickButton driverBack = new JoystickButton(driver, XboxController.Button.kBack.value);

    private static final JoystickButton coDriverA = new JoystickButton(coDriver, XboxController.Button.kA.value);
    private static final JoystickButton coDriverB = new JoystickButton(coDriver, XboxController.Button.kB.value);
    private static final JoystickButton coDriverX = new JoystickButton(coDriver, XboxController.Button.kX.value);
    private static final JoystickButton coDriverY = new JoystickButton(coDriver, XboxController.Button.kY.value);
    private static final JoystickButton coDriverLB = new JoystickButton(coDriver, XboxController.Button.kBumperLeft.value);
    private static final JoystickButton coDriverRB = new JoystickButton(coDriver, XboxController.Button.kBumperRight.value);
    private static final DPadButton coDriverDUp = new DPadButton(coDriver, DPadButton.Value.kDPadUp);
    private static final DPadButton coDriverDDown = new DPadButton(coDriver, DPadButton.Value.kDPadDown);
    private static final DPadButton coDriverDLeft = new DPadButton(coDriver, DPadButton.Value.kDPadLeft);
    private static final DPadButton coDriverDRight = new DPadButton(coDriver, DPadButton.Value.kDPadRight);
    private static final TriggerButton coDriverLT = new TriggerButton(coDriver, Hand.kLeft);
    private static final TriggerButton coDriverRT = new TriggerButton(coDriver, Hand.kRight);
    private static final JoystickButton coDriverStart = new JoystickButton(coDriver, XboxController.Button.kStart.value);
    private static final JoystickButton coDriverBack = new JoystickButton(coDriver, XboxController.Button.kBack.value);

    public static Timer jankTimer = new Timer();

    public static void init(){
        drive = new Drive();
        hood = new Hood();
        carousel = new Carousel();
        limelight = new Limelight();
        shooter = new Shooter();

        drive.setDefaultCommand(new DriveWithGamepad());
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private static void configureButtonBindings() {
        driverB.whenActive(new HoodHome());
        driverA.whenActive(new CarouselIntake()).whenInactive(new CarouselDefault());
        driverLT.whenActive(new CG_ReadyToFire());
        driverLB.whenActive(new CG_ShooterDefault());
//        driverRB.whenActive(new CG_FireReload());
        driverRT.whenActive(new CarouselShoot(0.075));
        driverY.whileActiveContinuous(new CarouselPurge());
        driverStart.and(driverBack).whenActive(new CG_Powerport());
//        driverX.whenActive(new DriveFollowPath("power_reverse")).whenInactive(new DriveFollowPath("power"));
//        driverX.whileActiveOnce(new DriveSetModuleRotation(90));

        coDriverB.whenActive(new HoodHome());
        coDriverA.whenActive(new CarouselIntake()).whenInactive(new CarouselDefault());
        coDriverLT.whenActive(new CG_ReadyToFire());
        coDriverLB.whenActive(new CG_ShooterDefault());
        coDriverRB.whenActive(new CG_FireReload());
        coDriverRT.whenActive(new CarouselShoot(0.1));
        coDriverY.whileActiveContinuous(new CarouselPurge());
//        driverLB.whenActive(new CG_SearchARed());
    }

    public static void setDriverRumble(double rumble){
        driver.setRumble(GenericHID.RumbleType.kLeftRumble, rumble);
        driver.setRumble(GenericHID.RumbleType.kRightRumble, rumble);
    }

    public static double getDriverLeftStickX() {
        if(Math.abs(driver.getX(Hand.kLeft)) < 0.1){
            return 0;
        }
        return squareStickInput(driver.getX(Hand.kLeft));
    }

    public static double getDriverLeftStickY() {
        if(Math.abs(driver.getY(Hand.kLeft)) < 0.1){
            return 0;
        }
        return squareStickInput(driver.getY(Hand.kLeft));
    }

    public static double getDriverRightStickX() {
        if(Math.abs(driver.getX(Hand.kRight)) < 0.1){
            return 0;
        }
        return squareStickInput(driver.getX(Hand.kRight));
    }

    public static double getDriverRightStickY() {
        if(Math.abs(driver.getY(Hand.kRight)) < 0.1){
            return 0;
        }
        return squareStickInput(driver.getY(Hand.kRight));
    }

    public static double squareStickInput(double input){
        if(input < 0){
            return -(input*input);
        }else{
            return input*input;
        }
    }

    public static double getDriverRightTrigger(){
        return driver.getTriggerAxis(Hand.kRight);
    }

    public static double getDriverLeftTrigger(){
        return driver.getTriggerAxis(Hand.kLeft);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public static Command getAutonomousCommand() {
        return null;
    }

    private static class TriggerButton extends Trigger {
        Hand hand;
        XboxController controller;

        public TriggerButton(XboxController controller, Hand hand) {
            this.hand = hand;
            this.controller = controller;
        }

        @Override
        public boolean get() {
            return controller.getTriggerAxis(hand) >= 0.5;
        }
    }

    private static class DPadButton extends Button {
        private final int dPadDegree;
        private final XboxController controller;

        public enum Value {
            kDPadRight, kDPadUpRight, kDPadUp, kDPadUpLeft, kDPadLeft, kDPadDownLeft, kDPadDown, kDPadDownRight,
        }

        /**
         * Creates a dpad button
         *
         * @param controller the controller to attach the button to
         * @param value      the dpad value
         */

        public DPadButton(XboxController controller, Value value) {
            this.controller = controller;
            switch (value) {
                case kDPadRight:
                    this.dPadDegree = 90;
                    break;
                case kDPadUpRight:
                    this.dPadDegree = 45;
                    break;
                case kDPadUp:
                    this.dPadDegree = 0;
                    break;
                case kDPadUpLeft:
                    this.dPadDegree = 315;
                    break;
                case kDPadLeft:
                    this.dPadDegree = 270;
                    break;
                case kDPadDownLeft:
                    this.dPadDegree = 225;
                    break;
                case kDPadDown:
                    this.dPadDegree = 180;
                    break;
                case kDPadDownRight:
                    this.dPadDegree = 135;
                    break;
                default:
                    throw new AssertionError("Illegal value" + value);
            }
        }

        @Override
        public boolean get() {
            return controller.getPOV() == dPadDegree;
        }
    }
}
