// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatzDriveTrainSubsystem;

public class DefaultDriveCommand extends CommandBase {

  private CatzDriveTrainSubsystem DRIVE_SUBSYSTEM;
  
  private final Supplier<Double> leftJoyXCmd, leftJoyYCmd, rightJoyXCmd, navXAngleCmd, pwrModeCmd;


  /** Creates a new DefaultDriveCommand. */
  public DefaultDriveCommand(CatzDriveTrainSubsystem drive, Supplier<Double> leftJoyXCmd, 
                                                            Supplier<Double> leftJoyYCmd, 
                                                            Supplier<Double> rightJoyXCmd, 
                                                            Supplier<Double> navXAngleCmd, 
                                                            Supplier<Double> pwrModeCmd   ) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive;
    this.leftJoyXCmd     = leftJoyXCmd;
    this.leftJoyYCmd     = leftJoyYCmd;
    this.rightJoyXCmd    = rightJoyXCmd;
    this.navXAngleCmd    = navXAngleCmd;
    this.pwrModeCmd      = pwrModeCmd;

    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double realTimeLeftJoyXCmd  = leftJoyXCmd.get();
    double realTimeLeftJoyYCmd  = leftJoyYCmd.get();
    double realTimeRightJoyXCmd = rightJoyXCmd.get();
    double realTimeNavXAngleCmd = navXAngleCmd.get();
    double realtimePwrModeCmd   = pwrModeCmd.get();

    DRIVE_SUBSYSTEM.cmdProcSwerve(realTimeLeftJoyXCmd, realTimeLeftJoyYCmd, realTimeRightJoyXCmd, realTimeNavXAngleCmd, realtimePwrModeCmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
