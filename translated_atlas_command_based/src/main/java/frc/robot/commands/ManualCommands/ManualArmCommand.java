// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatzArmSubsystem;

public class ManualArmCommand extends CommandBase 
{
  private final CatzArmSubsystem ARM_SUBSYSTEM;
  private Supplier<Double> triggerArmHoldPwr;
  private final double ARM_PWR_EXTEND = 0.2;
  private final double ARM_PWR_RETRACT = -0.2;
  private final double ARM_PWR_NONE = 0.0;
  /** Creates a new DefaultHoldingArmCommand. */
  public ManualArmCommand(CatzArmSubsystem ARM_SUBSYSTEM, 
                          Supplier<Double> triggerArmHoldPwr) 
  {
    this.ARM_SUBSYSTEM = ARM_SUBSYSTEM;
    this.triggerArmHoldPwr = triggerArmHoldPwr;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ARM_SUBSYSTEM);
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
    double armHoldPwr = triggerArmHoldPwr.get();

    if(armHoldPwr > 0.1)
    {
      ARM_SUBSYSTEM.setArmPwr(ARM_PWR_EXTEND);
    }
    else if(armHoldPwr < -0.1)
    {
      ARM_SUBSYSTEM.setArmPwr(ARM_PWR_RETRACT);
    }
    else
     ARM_SUBSYSTEM.setArmPwr(ARM_PWR_NONE);
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
