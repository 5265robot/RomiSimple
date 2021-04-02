// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// libraries that we need.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// ROMIINPUTS begins here
public class RomiInputs extends SubsystemBase {
  // create a variable for the light sensor
  private final AnalogPotentiometer m_lightSensor;

  // ROMIINPUTS instructions
  public RomiInputs() {
    /**
     * Make the light sensor. Notice that "Constants" has a squiggly red line
     * underneath it for you. That's because it needs to be added (imported) to the
     * libraries up above. Hover your mouse over it and you will see "Quick Fix".
     * Select the first option. Go to "Constants" so we can add our numbers.
     * 
     * 
     */
    m_lightSensor = new AnalogPotentiometer(Constants.AnalogInputs.lightSensor);
  }

  // ROMIINPUTS methods

  /**
   * "double" is for a decimal number with double precision. We return that value
   * with this command.
   */
  public double getLightSensor() {
    return m_lightSensor.get();
  }

  /**
   * "System.out.println" will write whatever we give it to the terminal on our
   * computer. In this case we will be able to see the output of the light sensor
   * to confirm what we are doing.
   * 
   * "void" means we don't return anything.
   */
  public void printSensor(){
    System.out.println(getLightSensor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
