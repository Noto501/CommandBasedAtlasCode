// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MiscCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatzIntakeSubsytem;
import frc.robot.subsystems.CatzStateMachineSubsystem;



public class IntakeRollersCommand extends CommandBase {
  /** Creates a new IntakeRollersCommand. */

  private CatzIntakeSubsytem INTAKE_SUBSYSTEM;
  private CatzStateMachineSubsystem STATE_MACHINE_SUBSYSTEM;

  private Supplier<Boolean> rightBumperPressed;
  private Supplier<Boolean> leftBumperPressed;
  
  public IntakeRollersCommand(CatzIntakeSubsytem INTAKE_SUBSYSTEM, CatzStateMachineSubsystem STATE_MACHINE_SUBSYSTEM, Supplier<Boolean> rightBumperPressed, 
                                                                                                                  Supplier<Boolean> leftBumperPressed) 
  {
    this.rightBumperPressed = rightBumperPressed;
    this.leftBumperPressed = leftBumperPressed;
    this.STATE_MACHINE_SUBSYSTEM = STATE_MACHINE_SUBSYSTEM;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(rightBumperPressed.get() == true)
    {
      if(STATE_MACHINE_SUBSYSTEM.getSelectedGamePiece() == 1)
      {
        INTAKE_SUBSYSTEM.rollersInCube();
      }
      else
      {
        INTAKE_SUBSYSTEM.rollersInCube();
      }
    }
    else if(leftBumperPressed.get() == true)
    {
      if(STATE_MACHINE_SUBSYSTEM.getSelectedGamePiece() == 1)
      {
        INTAKE_SUBSYSTEM.rollersOutCube();
      }
      else
      {
        INTAKE_SUBSYSTEM.rollersOutCube();
      }
    }
    else
    {
      INTAKE_SUBSYSTEM.rollersOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
