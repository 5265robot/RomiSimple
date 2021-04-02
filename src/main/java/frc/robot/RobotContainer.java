/**
 * LESSON 3 - let's make it driveable!
 * 
 * We're also adding a subsystem file, called RomiInputs.java, which will handle the input from the light sensor.
 * 
 * (and we'll clean up some unused parts of the code.)
 */

// libraries that we need
package frc.robot;

/**
 * notice that the next line is underlined. 
 * We never use this. 
 * You can either put two slashes in front of it or you can delete it. 
 * I'll delete it in the next lesson.
 */

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.subsystems.RomiInputs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// ROBOTCONTAINER begins here
public class RobotContainer {
  /**
   * first we declare our ingredients
   */
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  // we don't use m_autocommand - delete or comment it.
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_romiDrivetrain);
  private final XboxController m_xBox = new XboxController(0);
  // the 0 at the end is the USB port that the xBox controller gets plugged into
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final RomiInputs m_romiInput = new RomiInputs();

  // ROBOTCONTAINER instructions
  public RobotContainer() {
    // default driving will be arcade
    m_romiDrivetrain.setDefaultCommand(new RunCommand(
        () -> m_romiDrivetrain.arcadeDrive(-m_xBox.getRawAxis(1), m_xBox.getRawAxis(4)), m_romiDrivetrain));
    // configureButtonBindings() is defined below
    configureButtonBindings();
  }

  // ROBOTCONTAINER methods

  // Configure the button bindings
  private void configureButtonBindings() {
    /**
     * Right click (or two finger click) on subsystems and at the very bottom select
     * "Create a new class/command". Select "Subsystem (new)" and call the subsystem
     * RomiInputs. Follow the instructions in RomiInputs and then come back here to
     * put in the InstantCommand.
     * 
     * The InstantCommand will run the printSensor method from our instance of
     * RomiInputs, called m_romiInput
     */
    new JoystickButton(m_xBox, XboxController.Button.kB.value)
        .whenPressed(new InstantCommand(m_romiInput::printSensor, m_romiInput));
  }

  /**
   * By using a SendableChooser<Command> structure, we can pick from a list on the
   * computer to get the autonomous command we want. By typing "m_c" you should
   * only get "m_chooser" as a choice. By typing a "." after m_chooser you will
   * get a list of possible commands, including "getSelected()".
   * 
   * This is "public" because it is used in Robot.java during the autonomous
   * period of the game. If it was "private", Robot.java wouldn't be able to use
   * it.
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }

}
