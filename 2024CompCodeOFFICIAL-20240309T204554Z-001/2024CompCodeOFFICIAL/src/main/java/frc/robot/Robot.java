// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix.motorcontrol.ControlMode;


import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;



  private Timer timer = new Timer();
  private double[] times = new double[5];

  TalonFX shooter1 = new TalonFX(1); //og9 //outake motor SM 1/27
  TalonFX shooter2 = new TalonFX(2);
  TalonFX shooter3 = new TalonFX(14);
  TalonFX shooter4 = new TalonFX(15);
  TalonFX intake = new TalonFX(8); //intake motor SM 1/27
  TalonFX climber1 = new TalonFX(5); //hanger motor
  TalonFX climber2 = new TalonFX(11);

  Joystick stick = new Joystick(1);

  public Constants Robot = new Constants();


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    times[0] = 2;
    times[1] = 6;
    times[2] = 8;
    times[3] = 12;
    times[4] = 14;
    timer.reset();
    timer.start();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (timer.get() <= times[0]){
      shooter1.set(ControlMode.PercentOutput, 0.5); 
      shooter2.set(ControlMode.PercentOutput, 0.5);
      shooter3.set(ControlMode.PercentOutput, 0.5);
      shooter4.set(ControlMode.PercentOutput, 0.5);
    }
    //No motors run while moving for 4 secs
    else if (timer.get() <= times[1]){
      intake.set(ControlMode.PercentOutput, 0);
      shooter1.set(ControlMode.PercentOutput, 0);
      shooter2.set(ControlMode.PercentOutput, 0);
      shooter3.set(ControlMode.PercentOutput, 0);
      shooter4.set(ControlMode.PercentOutput, 0);
    }
    else if (timer.get() <= times[2]){
      intake.set(ControlMode.PercentOutput, 0.5);
    } 
    else if (timer.get() <= times[3]){
      intake.set(ControlMode.PercentOutput, 0);
      shooter1.set(ControlMode.PercentOutput, 0);
      shooter2.set(ControlMode.PercentOutput, 0);
      shooter3.set(ControlMode.PercentOutput, 0);
      shooter4.set(ControlMode.PercentOutput, 0);
    }
    else if (timer.get() <= times[4]){
      shooter1.set(ControlMode.PercentOutput, 0);
      shooter2.set(ControlMode.PercentOutput, 0);
      shooter3.set(ControlMode.PercentOutput, 0);
      shooter4.set(ControlMode.PercentOutput, 0);
    }
    else {
      shooter1.set(ControlMode.PercentOutput, 0);
      shooter2.set(ControlMode.PercentOutput, 0);
      shooter3.set(ControlMode.PercentOutput, 0);
      shooter4.set(ControlMode.PercentOutput, 0);
      intake.set(ControlMode.PercentOutput, 0.0);
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    boolean buttonA = stick.getRawButton(1);
    boolean buttonB = stick.getRawButton(2);
    boolean buttonX = stick.getRawButton(3);
    boolean buttonY = stick.getRawButton(4);

    boolean LB = stick.getRawButton(5);
    boolean RB = stick.getRawButton(6);

    double triggerL = stick.getRawAxis(2);
    double triggerR = stick.getRawAxis(3);

    //Climbers
    if (buttonX == true) {
      climber1.set(ControlMode.PercentOutput, 0.2);
      climber2.set(ControlMode.PercentOutput, -0.2);
    }
    else if (buttonY == true) {
      climber1.set(ControlMode.PercentOutput, -0.2);
      climber2.set(ControlMode.PercentOutput, 0.2);
    }
    else {
      climber1.set(ControlMode.PercentOutput, 0);
      climber2.set(ControlMode.PercentOutput, 0);
    }

    //Intake
    if (buttonA == true) { //When 'A' gets pressed, motor runs at 20% clockwise
      intake.set(ControlMode.PercentOutput, 0.7);
    }
    else if (buttonB == true){ //When 'B' gets pressed, motor runs at 20% counterclockwise
      intake.set(ControlMode.PercentOutput, -0.7);
    }
    else {
      intake.set(ControlMode.PercentOutput, 0);
    } 

    //Outtake/Shooter
    if (triggerR > 0.05 ){
      shooter1.set(ControlMode.PercentOutput, -0.8);
      shooter2.set(ControlMode.PercentOutput, -0.8);
      shooter3.set(ControlMode.PercentOutput, 0.8);
      shooter4.set(ControlMode.PercentOutput, 0.8);
      intake.set(ControlMode.PercentOutput, -0.8);
    }
    else if (RB == true){
      shooter1.set(ControlMode.PercentOutput, 0.8);
      shooter2.set(ControlMode.PercentOutput, 0.8);
      shooter3.set(ControlMode.PercentOutput, -0.8);
      shooter4.set(ControlMode.PercentOutput, -0.8);
      intake.set(ControlMode.PercentOutput, 0.8);

    }
    else{
      shooter1.set(ControlMode.PercentOutput, 0);
      shooter2.set(ControlMode.PercentOutput, 0);
      shooter3.set(ControlMode.PercentOutput, 0);
      shooter4.set(ControlMode.PercentOutput, 0);
      intake.set(ControlMode.PercentOutput, 0);

    }
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
