// libraries that we need
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.Drive;
import frc.robot.Constants.path;
import frc.robot.Constants.traj;
import frc.robot.Constants.Drive.Auto;
import frc.robot.commands.drnDriveByEncoder;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// ROBOTCONTAINER begins here
public class RobotContainer {
  // drn -- robot's subsystems and commands are defined here...

  // drn -- drive subsystem declarations
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();

  // drn --- declaring an instance of the XBox controller
  private final XboxController m_xBox = new XboxController(Constants.IO.Controller);

  // drn -- A chooser for autonomous commands
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // drn -- subsystem for distance sensor
  //private final RomiInputs m_romiInput = new RomiInputs();

  // drn -- trajectory variables
  // controller -- ramsete
  RamseteController m_rController = new RamseteController(traj.kRamseteB, traj.kRamseteZeta);
  // feed forward -- motor
  SimpleMotorFeedforward m_Feedforward = new SimpleMotorFeedforward(
    traj.ksVolts,traj.kvVoltSecondsPerMeter,traj.kaVoltSecondsSquaredPerMeter);
    
  

  // drn -- test command for autonomous
  // drives forward for 2 secs
  private final Command SimpleDriveForward = new RunCommand( () ->
    m_romiDrivetrain.curveDrive(Auto.simpleDriveSpeed, 0.0),m_romiDrivetrain)
    .withTimeout(Auto.simpleDriveTime);

  private final Command SimpleDriveCircle = new RunCommand( () ->
    m_romiDrivetrain.curveDrive(Auto.simpleDriveSpeed, Auto.simpleDriveTurn),m_romiDrivetrain)
    .withTimeout(Auto.simpleDriveTime);


  // ROBOTCONTAINER instructions
  public RobotContainer() {

    // default driving will be arcade drive
    m_romiDrivetrain.setDefaultCommand(new RunCommand(
      () -> m_romiDrivetrain.arcadeDrive(-m_xBox.getRawAxis(1), m_xBox.getRawAxis(4)), m_romiDrivetrain));

    // drn -- sets up the driver's station to have options for autonomous
    m_chooser.setDefaultOption("auto forward", SimpleDriveForward);
    m_chooser.addOption("rotate in place", SimpleDriveCircle);
    m_chooser.addOption("auto paths", autoRamsetePaths());
    // drn - publish auto options
    SmartDashboard.putData(m_chooser);

    // configureButtonBindings() is defined below
    configureButtonBindings();
  }

  // ROBOTCONTAINER methods

  // Configure the button bindings
  private void configureButtonBindings() {
    SmartDashboard.setDefaultNumber("Spin Volts", Auto.simpleSpinVolt);
    SmartDashboard.setDefaultNumber("Spin timeout", Auto.simpleSpinTimeout);

    new JoystickButton(m_xBox, XboxController.Button.kB.value)
        .whenPressed(
          new InstantCommand(() -> m_romiDrivetrain.tankDriveVolts(
              SmartDashboard.getNumber("Spin Volts", Auto.simpleSpinVolt),SmartDashboard.getNumber("Spin Volts", Auto.simpleSpinVolt)),
              m_romiDrivetrain)
            .withTimeout(SmartDashboard.getNumber("Spin timeout", Auto.simpleSpinTimeout))
          );

    new JoystickButton(m_xBox, XboxController.Button.kA.value)
          .whenPressed(new drnDriveByEncoder(100.0, 100.0));

    new JoystickButton(m_xBox, XboxController.Button.kX.value)
          .whenPressed(printEncoders());

}

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }

  // X X X X X X X X X X X 
  // TRAJECTORY methods

  public DifferentialDriveVoltageConstraint setAutoVoltageConstraint(){
    return new DifferentialDriveVoltageConstraint(
      m_Feedforward, traj.kDriveKinematics,10);
  }
  public TrajectoryConfig setTrajConfig(){
    return new TrajectoryConfig(Drive.Auto.kSpeed,Drive.Auto.kAccel)
      .setKinematics(traj.kDriveKinematics)
      .addConstraint(setAutoVoltageConstraint());
  }
  public Trajectory setTrajA(){
    return TrajectoryGenerator.generateTrajectory(path.A.startPose, path.A.points, path.A.endPose, setTrajConfig());
  }
  public Trajectory setTrajB(){
    return TrajectoryGenerator.generateTrajectory(path.B.startPose, path.B.points, path.B.endPose, setTrajConfig());
  }
  public Trajectory setTrajC(){
    return TrajectoryGenerator.generateTrajectory(path.C.startPose, path.C.points, path.C.endPose, setTrajConfig());
  }
  public Trajectory setTrajD(){
    return TrajectoryGenerator.generateTrajectory(path.D.startPose, path.D.points, path.D.endPose, setTrajConfig());
  } 
  public RamseteCommand ramsetePath (Trajectory thisPath){
    return new RamseteCommand(thisPath, m_romiDrivetrain::getPose, 
    m_rController, m_Feedforward, traj.kDriveKinematics,
    m_romiDrivetrain::getWheelSpeeds, new PIDController(traj.kPDriveVel, 0, 0), 
    new PIDController(traj.kPDriveVel, 0, 0), m_romiDrivetrain::tankDriveVolts, m_romiDrivetrain);
  }

  public Command autoRamsetePaths(){
    return new InstantCommand(() -> m_romiDrivetrain.resetOdometry(setTrajA().getInitialPose()),m_romiDrivetrain)
    .andThen( ramsetePath(setTrajA()) )
    //.andThen(new InstantCommand(() -> ramsetePath(setTrajA())))
    //.andThen( ramsetePath(setTrajB()) )
    //.andThen( ramsetePath(setTrajC()) )
    //.andThen( ramsetePath(setTrajD()) )
    .andThen(new InstantCommand(() -> m_romiDrivetrain.fullStop(),m_romiDrivetrain));
  }

  public Command printEncoders (){
    System.out.println("test string");
    return null;
  }
  
}
