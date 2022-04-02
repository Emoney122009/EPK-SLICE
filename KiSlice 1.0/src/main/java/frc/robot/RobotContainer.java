// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/**
 * THINGS TO DO:
 * CHANGE ENCODER TO METERS FOR AUTO
 * CHANGE PIDS
 * CHANGE RPM CURVE TO TRAPAZOIDAL
 * ETC
 *
 */


package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.Constants.c;
import frc.robot.Constants.id;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Climber.*;
import frc.robot.commands.DriveTrain.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Indexer.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Climber sub_climber = new Climber();
  private final DriveTrain sub_drive = new DriveTrain();
  private final Shooter sub_shooter = new Shooter();
  private final Indexer sub_indexer = new Indexer();
  private final Intake sub_intake = new Intake();

  private final Joystick hid_lJoy = new Joystick(id.lJoy);
  private final Joystick hid_rJoy = new Joystick(id.rJoy);
 // private final XboxController hid_xbox = new XboxController(id.xbox);
 // private final XboxController hid_xboxExtra = new XboxController(5); // for testing

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
 
    sub_drive.setDefaultCommand(new ArcadeDrive(sub_drive, () -> -hid_rJoy.getY() , () -> hid_lJoy.getX(), c.xMod, c.yMod));
    //sub_intake.setDefualtCommand(new IntakeSetSpeed(sub_intake, 1));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * 
   * 
   * SUPER TEMP/ NOT FOR IRL USE
   * 
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(hid_lJoy, 3).whileActiveOnce(new SetClimberSpeed(sub_climber, 0.6));
    new JoystickButton(hid_lJoy, 4).whileActiveOnce(new SetClimberSpeed(sub_climber, -0.6));
    new JoystickButton(hid_lJoy, 5).whileActiveOnce(new SetClimberHeight(sub_climber, 2));
    new JoystickButton(hid_lJoy, 6).whileActiveOnce(new SetClimberHeight(sub_climber, 0));
    new JoystickButton(hid_rJoy, 4).whileActiveOnce(new IndexerSetSpeed(sub_indexer, .6));
    new JoystickButton(hid_rJoy, 5).whileActiveOnce(new IndexerSetSpeed(sub_indexer, -.6));
    new JoystickButton(hid_rJoy, 1).whileActiveOnce(new IntakeSetSpeed(sub_intake, 0.5));
    new JoystickButton(hid_lJoy, 1).whileActiveOnce(new AutoShoot(sub_shooter));
    //new JoystickButton(hid_lJoy, 12).whileActiveOnce(new ClimberMotorMode(sub_climber));
    new JoystickButton(hid_lJoy, 11).whileActiveOnce(new DriveFoward(sub_drive, 100));
    new JoystickButton(hid_lJoy, 12).whileActiveOnce(new DriveFoward(sub_drive, -100));
    new JoystickButton(hid_lJoy, 10).whileActiveOnce(new SetClimberHeight(sub_climber, 360));
    new JoystickButton(hid_lJoy, 9).whileActiveOnce(new SetClimberHeight(sub_climber, 0));
    new JoystickButton(hid_lJoy, 8).whileActiveOnce(new SetLeftClimber(sub_climber,-10));
    new JoystickButton(hid_lJoy, 7).whileActiveOnce(new SetLeftClimber(sub_climber,10));
    new JoystickButton(hid_rJoy, 8).whileActiveOnce(new SetRightClimber(sub_climber,-10));
    new JoystickButton(hid_rJoy, 7).whileActiveOnce(new SetRightClimber(sub_climber,10));
    new JoystickButton(hid_lJoy, 2).whileActiveOnce(new movePneumatics(sub_climber));

    
  }




  /**
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
     return new SequentialCommandGroup(
        new DriveFoward(sub_drive, 10),
        new ParallelCommandGroup(
          new IntakeSetSpeed(sub_intake, .5),
          new IndexerSetSpeed(sub_indexer, .5)
        )
      );

    //m_autonomous
  }
}

