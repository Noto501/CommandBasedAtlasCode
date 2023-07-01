// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



//---------------------------------------------------------------------
// This subsystem contains the functions for determining what gamepiece is in play
//This is determined through an int called selectedGamePiece
//---------------------------------------------------------------------


package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CatzStateMachine extends SubsystemBase{

  private int selectedGamePiece;

  public CatzStateMachine() 
  {
    
  }

  public void periodic()
  {

  }
  
  //sets the selected Gamepiece to 'No gamepiece selected'
  public void setStateNone() 
  {
    selectedGamePiece = 0;
  }

  //sets the selected Gamepiece to CUBE
  public void setStateCube()
  {
    selectedGamePiece = 1;
  }

  //sets the selected Gampiece to CONE
  public void setStateCone()
  {
   selectedGamePiece = 2;
  }

  //returns the gamepieces int through a return function if called
  public int getSelectedGamePiece()
  {
    return selectedGamePiece;
  }
}
