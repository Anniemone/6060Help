package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        AutoBuilder.configureHolonomic(
                                        s_Swerve::getPose, // Robot pose supplier
                                        s_Swerve::resetOdometry, // Method to reset odometry (will be
                                                                                // called if your auto
                                                                                // has a starting pose)
                                        s_Swerve::getRobotVelocity, // ChassisSpeeds supplier. MUST BE
                                                                                        // ROBOT RELATIVE
                                        s_Swerve::drive, // Method that will drive the robot
                                                                                        // given ROBOT
                                                                                        // RELATIVE ChassisSpeeds
                                        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
                                                                        // in your
                                                                        // Constants class
                                                        new PIDConstants(translationAxis, strafeAxis, rotationAxis),
                                                        // Translation PID constants
                                                        new PIDConstants(translationAxis, strafeAxis, rotationAxis),
                                                        // Rotation PID constants
                                                        4.5,
                                                        // Max module speed, in m/s
                                                        Math.sqrt((Constants.Swerve.wheelBase/2)*(Constants.Swerve.wheelBase/2)+(Constants.Swerve.wheelBase/2)*(Constants.Swerve.wheelBase/2)),
                                                        // Drive base radius in meters. Distance from robot center to furthest
                                                        // module.
                                                        new ReplanningConfig()
                                        // Default path replanning config. See the API for the options here
                                        ),
                                        () -> {
                                                // Boolean supplier that controls when the path will be mirrored for the red
                                                // alliance
                                                // This will flip the path being followed to the red side of the field.
                                                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                                                var alliance = DriverStation.getAlliance();
                                                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red
                                                                : false;
                                        },
                                        s_Swerve); // Reference to this subsystem to set requirements
                        autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Competition", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
