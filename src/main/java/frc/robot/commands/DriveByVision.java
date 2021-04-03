// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ROMIChassis;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriveByVision extends CommandBase {
//  NetworkTableEntry forward;
//  NetworkTableEntry leftright;
//  NetworkTableEntry stop;

//  double forwardspeed = 0.0;
//  double leftrightturn = 0.0;

  ROMIChassis chassis;

  /** Creates a new DriveByVision. */
  public DriveByVision() {
    chassis = RobotContainer.chassis;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Initial the Network Table and grab the SmartDashboard -> DriveByVision table    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("DriveByVision");
    NetworkTableEntry forward_speed = table.getEntry("Forward Speed");
    NetworkTableEntry leftright_turn = table.getEntry("Left-Right");
    NetworkTableEntry quickturn = table.getEntry("QuickTurn");

    double f = forward_speed.getDouble(0.0);
    double lr = leftright_turn.getDouble(0.0);
    boolean qt = quickturn.getBoolean(true);

    chassis.DBV(f, lr, qt);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("DriveByVision");
    NetworkTableEntry stop = table.getEntry("Stop");
    boolean s = stop.getBoolean(false);
    return s;
  }
}
