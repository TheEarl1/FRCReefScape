/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.WoS;
import frc.robot.commands.WoS.*;
import frc.robot.commands.elevator.Jesus;
import frc.robot.commands.elevator.Lucifer;
import frc.robot.commands.funnel.Explode;
import frc.robot.commands.funnel.Implode;
import frc.robot.commands.AlgaeIntake.Nibble;
import frc.robot.commands.AlgaeIntake.Spit;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.SwerveDriveInputs;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private static OI m_oi;

  private static XboxController m_DriverXboxController;
  private static XboxController m_OperatorXboxController;

  private SwerveDriveInputs m_driveInputs;

  /**
   * Used outside of the OI class to return an instance of the class.
   * @return Returns instance of OI class formed from constructor.
   */
  public static OI getInstance() {
    if (m_oi == null) {
      m_oi = new OI();
    }
    return m_oi;
  }

  public OI() {
    // User Input
    // TODO: Tune deadband
    m_DriverXboxController = new XboxController(RobotMap.U_DRIVER_XBOX_CONTROLLER);
    m_OperatorXboxController = new XboxController(RobotMap.U_OPERATOR_XBOX_CONTROLLER);

    // Set up drive translation and rotation inputs
    XboxController driveController = m_DriverXboxController;
    Supplier<Double> xInput;
    Supplier<Double> yInput;
    if(RobotBase.isReal()){
      xInput = ()->driveController.getLeftY();
      yInput = ()->driveController.getLeftX();
    } else {
      xInput = ()->-driveController.getLeftX();
      yInput = ()->driveController.getLeftY();
    }
    m_driveInputs = new SwerveDriveInputs(xInput, yInput, ()->driveController.getRightX());
  }

  public void bindControls() {
    ////////////////////////////////////////////////////
    // Now Mapping Commands to XBox
    ////////////////////////////////////////////////////

    // Driver Grease Man's Special Abilities(OP)
    new JoystickButton(m_DriverXboxController, Button.kBack.value).onTrue(new InstantCommand(()->Drive.getInstance().zeroHeading()));

    //Operator Cookie Monster Special Abilities(MEGA OP)
    new JoystickButton(m_OperatorXboxController, Button.kB.value).whileTrue(new Jesus());
    new JoystickButton(m_OperatorXboxController, Button.kA.value).whileTrue(new Lucifer());
    new JoystickButton(m_OperatorXboxController, Button.kX.value).whileTrue(new Implode());
    new JoystickButton(m_OperatorXboxController, Button.kY.value).whileTrue(new Explode());
   
    new Trigger(()->{return (m_OperatorXboxController.getLeftTriggerAxis() > 0.5);}).whileTrue(new Consume());
    new Trigger(()->{return (m_OperatorXboxController.getRightTriggerAxis() > 0.5);}).whileTrue(new Expel());
    new JoystickButton(m_OperatorXboxController, Button.kX.value).whileTrue(new Nibble());
    new JoystickButton(m_OperatorXboxController, Button.kY.value).whileTrue(new Spit());
  };
  
    
  

  /**
   * Returns the Xbox Controller
   * @return the Xbox Controller
   */
  public XboxController getDriverXboxController() {
      return m_DriverXboxController;
  }

  public XboxController getOperatorXboxController() {
    return m_OperatorXboxController;
  }

  public SwerveDriveInputs getDriveInputs() {
    return m_driveInputs;
  }
}

