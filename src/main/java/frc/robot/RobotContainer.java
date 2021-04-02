/**
 * LESSON 1 - What does all this stuff mean? What is it used for?
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This is a definition of what a RobotContainer should be
 */
public class RobotContainer {
  /**
   * first we declare our ingredients
   */
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  /**
   * private - this object can only be seen in this file
   * 
   * final - this object's name will never point to something else
   * 
   * RomiDrivetrain is the name of a subsystem - you can see it in the file list.
   * But it is just a definition, not an object. When we make an "instance" of
   * RomiDrivetrain, we need to "declare" what it is (in this case the object is a
   * RomiDrivetrain) and give the instance a name, such as "m_romiDrivetrain".
   * 
   * new - we are making a new instance
   * 
   * RomiDriveTrain() - notice the parantheses!! - is the first method in the
   * RomiDrivetrain subsystem and containes the instructions for making a
   * RomiDrivetrain
   */

  /**
   * CONVENTIONS for making names
   * 
   * m_ means my, mine, this is my instance
   * 
   * CamelCase (the first letter of each word, including the first word, is
   * uppercase). We use this for the names of definitions, also called "types".
   * Most things in CamelCase come with the program.
   * 
   * lowerCamelCase (not an uppercase Lower, but a lowercase lower). We use this
   * for the names of instances of objects. Most of the things you'll make should
   * follow the convention: m_lowerCamelCase
   */

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_romiDrivetrain);

  /**
   * I prefer this example:
   * 
   * private final ChocolateChipCookie m_chocolateChipCookie = new
   * ChocolateChipCookie();
   * 
   * you can eat m_chocolateChipCookie after following the ChocolateChipCookie()
   * recipe that is found on the ChocolateChipCookie page.
   */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * This container is also the recipe for making a m_robotContainer. (That
   * happens in Robot.java)
   * 
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
