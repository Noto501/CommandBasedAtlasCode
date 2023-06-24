// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatzElevatorSubsystem;


public class ManualElevatorCommand extends CommandBase {
  private final CatzElevatorSubsystem ELEVATOR_SUBSYSTEM;

  private Supplier<Double> elevatorHoldPwr;
  private Supplier<Boolean> fullManualActiveFunction;

  
  private final double MANUAL_CONTROL_DEADBAND = 0.1;
  private final double MANUAL_HOLD_STEP_SIZE = 10000.0; //5000.0;

  /** Creates a new DefaultHoldingPosCommand. */
  public ManualElevatorCommand(CatzElevatorSubsystem ELEVATOR_SUBSYSTEM, Supplier<Double> elevatorHoldPwr, 
                                                                         Supplier<Boolean> fullManualActiveFunction)
  {
    this.ELEVATOR_SUBSYSTEM = ELEVATOR_SUBSYSTEM;
    this.elevatorHoldPwr = elevatorHoldPwr;
    this.fullManualActiveFunction = fullManualActiveFunction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ELEVATOR_SUBSYSTEM);
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
    boolean elevatorInManual = fullManualActiveFunction.get();
    double elevatorPwr = elevatorHoldPwr.get();
    double targetPositionEnc;

    if(Math.abs(elevatorPwr) >= MANUAL_CONTROL_DEADBAND)
    {
        if(elevatorInManual) // Full manual
        {

            ELEVATOR_SUBSYSTEM.elevatorManual(elevatorPwr);
        }
        else // Hold Position
        {
            targetPositionEnc = ELEVATOR_SUBSYSTEM.getElevatorEncoder();
            targetPositionEnc = targetPositionEnc + (elevatorPwr * MANUAL_HOLD_STEP_SIZE);

            ELEVATOR_SUBSYSTEM.elevatorHoldingManual(targetPositionEnc);
        }
    }
    else
    {
        if (elevatorInManual) //Full manual but no pwr applied
        {
            ELEVATOR_SUBSYSTEM.elevatorManual(0.0);
        }
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
