// libraries that we need
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.subsystems.RomiInputs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// ROBOTCONTAINER begins here
public class RobotContainer {
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  private final XboxController m_xBox = new XboxController(Constants.IO.Controller);
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
    // testing the light sensor
    new JoystickButton(m_xBox, XboxController.Button.kB.value)
        .whenPressed(new InstantCommand(m_romiInput::printSensor, m_romiInput));
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }

}
