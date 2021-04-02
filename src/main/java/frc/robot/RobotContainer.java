/**
 * LESSON 2 - let's make it driveable!
 * 
 * We're also adding two files, a command called ArcadeDrive.java and a subsystem called RomiInputs.java.
 * 
 * (and remove lots of the unnecessary comments from WPI and from me)
 */

// libraries that we need
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.RomiDrivetrain;
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
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_romiDrivetrain);

  /**
   * NEW INGREDIENTS: the xBox controller and a selection box
   */
  private final XboxController m_xBox = new XboxController(0);
  // the 0 at the end is the USB port that the xBox controller gets plugged into
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * We'll add commands to m_chooser, and then we can have those commands as
   * choices for our autonomous routine. But that's for the next lesson.
   */

  // ROBOTCONTAINER instructions
  public RobotContainer() {
    /*
     * lets make the default style of driving to arcade style. There is already a
     * command for that in RomiDrivetrain.
     * 
     * Runcommand() is a convenience command. Convenient doesn't mean easy to
     * understand with things such as "() ->". I'll go into this in the next lesson.
     */
    m_romiDrivetrain.setDefaultCommand(new RunCommand(
        () -> m_romiDrivetrain.arcadeDrive(-m_xBox.getRawAxis(1), m_xBox.getRawAxis(4)), m_romiDrivetrain));
    // configureButtonBindings() is defined below
    configureButtonBindings();
  }

  // ROBOTCONTAINER methods

  // Configure the button bindings
  private void configureButtonBindings() {
    /**
     * Make a button that reads the light sensor. This doesn't do anything until we
     * put the method into the InstandCommand()
     * 
     * By typing "m_x" you should only get "m_xBox" as a choice. By typing a "."
     * after XboxController, you'll see "Button" as a choice. Another "." and you'll
     * see all the buttons. And we need the "value" of the button. JoystickButton
     * has lots of methods associated with it, and you get those by typing a ".".
     * We'll use "whenPressed()".
     * 
     * I'll describe what an InstantCommand() is next lesson.
     */
    new JoystickButton(m_xBox, XboxController.Button.kB.value).whenPressed(new InstantCommand());
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
