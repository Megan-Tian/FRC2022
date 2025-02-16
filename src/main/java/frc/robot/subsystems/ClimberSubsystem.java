/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeCommand;

/**
 * Add your docs here.
 */
public class ClimberSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static final int PH_CAN_ID = 15;
  PneumaticHub m_ph;

  DoubleSolenoid climberSolenoid;

  public ClimberSubsystem() {

    m_ph = new PneumaticHub(PH_CAN_ID);
    // climberSolenoid = m_ph.makeDoubleSolenoid(RobotMap.CLIMBER_FORWARD_CHANNEL, RobotMap.CLIMBER_REVERSE_CHANNEL);
    climberSolenoid = m_ph.makeDoubleSolenoid(3,2);


  }

  public void extendClimber() {
    climberSolenoid.set(Value.kReverse); // "forward" is 15 open
    
  }

  public void retractClimber() {
    climberSolenoid.set(Value.kForward);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
