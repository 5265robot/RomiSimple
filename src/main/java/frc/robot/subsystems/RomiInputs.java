// libraries that we need.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// ROMIINPUTS begins here
public class RomiInputs extends SubsystemBase {
  // light sensor
  private final AnalogPotentiometer m_lightSensor;

  // ROMIINPUTS instructions
  public RomiInputs() {
    // create light sensor instance
    m_lightSensor = new AnalogPotentiometer(Constants.AnalogInputs.lightSensor);
  }

  // ROMIINPUTS methods

  // get the value for the light sensor
  public double getLightSensor() {
    return m_lightSensor.get();
  }

  // give a value to turn the robot
  public double getTurnRate() {
    double diff = m_lightSensor.get();
    return diff;
  }

  // put the value in the terminal
  public void printSensor() {
    System.out.println(getLightSensor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // put the light sensor on the SmartDashboard
    SmartDashboard.putNumber("light", getLightSensor());
  }
}
