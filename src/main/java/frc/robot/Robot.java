// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.extendSub;
import frc.robot.subsystems.gripperSub;
import frc.robot.subsystems.rotationSub;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //making drivetrain as function for m_chooser
  private final driveTrain mdrivetrain = new driveTrain();

  
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public String m_autoselected;
  SendableChooser<String> m_chooser = new SendableChooser<>();


  private static final String KGOALONLY = "Goal!";

  public static int autoStatus = 0;


  
  // Creates a new AHRS object under ahrs.
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //Motors and motorcontroller settings initialize on startup
    driveTrain.driveSettings();
    extendSub.ExtenderSRXsettings();
    gripperSub.GripperSRXsettings();
    rotationSub.RotaterSRXsettings();
  //NOT in robotcontainer due to only one run needed


    // schedule the autonomous command
   //add on to dashboard to run autonomous functionally 
  
   SmartDashboard.putData("Auton Choice", m_chooser);
   


  /*
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
  */}
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
    autoStatus = 0;
    m_autonomousCommand = m_robotContainer.getAuton();

    if(m_autonomousCommand != null){
      m_autonomousCommand.schedule();
    }

     
  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    }
     
  

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. 
    if(m_autonomousCommand != null){
      m_autonomousCommand.cancel();
    }
    
   mdrivetrain.driveRotations(5);
   


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
