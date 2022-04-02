/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.models.VisionObject;

// import com.kauailabs.navx.frc.AHRS;  
// import edu.wpi.first.wpilibj.SPI;

/**
 * Copy of FetchPowerCellCommand with modified contructor to take cargo color.
 * <p>
 * Drives to closest cargo. CLOSED-LOOP.
 */

public class FetchCargoCommand extends Command {
  PIDController strafeController;
  PIDController forwardController;
  double gyroAngle;
  double angle;
  double desiredAngle;
  double setPointAngle = 6;
  boolean isClose;
  double forwardControllerVelocity = -0.5; 

  String cargoColor; // blue or red, gets passed into constructor
  VisionObject closestObject;

  public double totalRotation = 0;

  public FetchCargoCommand(String cargoColor) {
    requires(Robot.drivetrainSubsystem);
  }

  public FetchCargoCommand(String cargoColor, double timeout) {
    super(timeout);
    requires(Robot.drivetrainSubsystem);
  }

  public FetchCargoCommand(String cargoColor, double timeout, double forwardVelocity) {
    super(timeout);
    requires(Robot.drivetrainSubsystem);
    this.forwardControllerVelocity = forwardVelocity; 
  }

  protected void initPID() {
    strafeController = new PIDController(0.011, 0.0, 0.0); // TODO update constants
    forwardController = new PIDController(0.05, 0.01, 0.0); // TODO update constants
  }

  @Override
  protected void initialize() {
    initPID();
    checkUpdateCargoColor(this.cargoColor);
    System.out.println("FCC start");

    // SmartDashboard.putNumber("Vision angle", angle);
    // SmartDashboard.putNumber("Desired angle", desiredAngle);
    // SmartDashboard.putNumber("initial angle", gyroAngle);
    // SmartDashboard.putNumber("SetPoint angle", setPointAngle);

    Robot.drivetrainSubsystem.resetKinematics(Vector2.ZERO, 0);
    System.out.println("Initialized FCC");

    isClose = false;
  }

  @Override
  protected void execute() {
    // repeatedly gets the nearest cargo of the given color
    Robot.objectTrackerSubsystem.data();
    closestObject = Robot.objectTrackerSubsystem.getClosestObject(cargoColor);

    double forward = 0;
    double strafe = 0;

    // quits command if no objects are in frame
    if (closestObject == null) {
      SmartDashboard.putNumber("driveRotation", 99);
      Robot.drivetrainSubsystem.holonomicDrive(new Vector2(0, 0), 0.0, false);
      return;
    }

    // System.out.println("Closest z: " + closestObject.z);
    // latenncy does need to be remeasured so this compensation may not be 100% accurate
    closestObject.motionCompensate(Robot.drivetrainSubsystem, true); 

    // STRAFE
    // strafeController.setSetpoint(closestObject.x) sets a POSIITON setpoint, even though it is later passed into holonomicDrive() as a VELOCITY
    strafeController.setSetpoint(closestObject.x); 
    strafe = strafeController.calculate(0);

    if (strafe > 1) {
      strafe = 1;
    } else if (strafe < -1) {
      strafe = -1;
    }

    SmartDashboard.putNumber("driveStrafe", strafe);

    // FORWARD
    // Forward velocity is set contant below and does NOT depend on position
    // forwardController.setSetpoint(closestObject.z-RobotMap.TARGET_TRIGGER_DISTANCE);
    // TODO figure out how to implement code that begins intake process
    forward = forwardController.calculate(0);
    
    if (forward > 1) {
      forward = 1;
    } else if (forward < -1) {
      forward = -1;
    }

    SmartDashboard.putNumber("driveForward", forward);

    final boolean robotOriented = false;

    /* holonomicDrive() takes a vector2 VELOCITY - NOT position
     * explains why there are checks that set forward and strafe to <1 and >-1 to
     * keep within motor speed range
     */

    final Vector2 translation = new Vector2(forwardControllerVelocity, strafe);

    // System.out.println("translation: " + translation);
    Robot.drivetrainSubsystem.holonomicDrive(translation, 0.0, robotOriented);
  }

  @Override
  protected boolean isFinished() {
    double tolerance = 4; // TODO units...? i think it's inches
    if (closestObject == null) {
      return false;
    } // TODO could lose sight for small amount of time causing command to finish early
    
    isClose = Math.abs(closestObject.z - RobotMap.TARGET_TRIGGER_DISTANCE) <= tolerance;

    if (isClose) {
      System.out.println("FCC done");
    }

    return isClose;
    // TODO: add the actual completion test code
  }

  @Override
  protected void end() {
    // Robot.vision.ledOff();
    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0, true);
    System.out.println("FCC end()");
  }

  // Called when another command which requires one or more of the same subsystems
  // is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  private void checkUpdateCargoColor(String color) {
    // checks cargoColor given to constructor, normalizes case
    if (color.equalsIgnoreCase("blue")) {
      this.cargoColor = "blue";
    } else if (color.equalsIgnoreCase("red")) {
      this.cargoColor = "red";
    } else {
      // TODO some time of catch all end case that deals with a non red/blue input?
    }
  }
}
