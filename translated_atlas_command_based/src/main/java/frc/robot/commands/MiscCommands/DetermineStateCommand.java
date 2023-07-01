// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MiscCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatzStateMachine;

public class DetermineStateCommand extends CommandBase {
  private int selectedGamePiece;
  private CatzStateMachine STATE_MACHINE_SUBSYSTEM;
  /** Creates a new DetermineStateCommand. */
  public DetermineStateCommand(int selectedGamePiece, CatzStateMachine STATE_MACHINE_SUBSYSTEM) 
  {
    this.selectedGamePiece = selectedGamePiece;
    this.STATE_MACHINE_SUBSYSTEM = STATE_MACHINE_SUBSYSTEM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(STATE_MACHINE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if(selectedGamePiece == 0)
    {
      STATE_MACHINE_SUBSYSTEM.setStateNone();
    }
    else if(selectedGamePiece == 1)
    {
      STATE_MACHINE_SUBSYSTEM.setStateCube();
    }
    else
    {
      STATE_MACHINE_SUBSYSTEM.setStateCone();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
