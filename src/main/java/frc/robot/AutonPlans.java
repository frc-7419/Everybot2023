/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AutonPlans extends TimedRobot {

  private RobotContainer robotContainer;

  

  @Override
  public void autonomousInit() {
    /*
     * Right Most Position From Alliance Area:
     * If Cube Is Preloaded
        * Sequentially:
        *  Turn -90 degrees (to the left)
        *  Move 1ft
        *  Turn -90 degrees (to the left)
        *  Parallel:
        *    Raise Arm
        *     Move 2ft
        *  Lower Arm (Preloaded Game Piece scored)

        If Cone Is Preloaded
        * Sequentially:
        *  Turn -180 degrees 
        *  Parallel:
        *    Raise Arm
        *     Move 2ft
        *  Lower Arm (Preloaded Game Piece scored)

        AFTER PRELOAD PIECE SCORED:
        Turn 180 Degrees
        Move 10ft
        Turn -90 degrees (to the left)
        Move 4.5ft
        Turn -90 degrees (to the left)
        Move 2.5ft onto Charge Station
        Run SmartBalance
            Smart Balance: Moves backwards and forwards until the gyro
            is within 2.5 degrees level of it's original X Axis rotation

          - - - - - - - - - - 
        * Left Most Position From Alliance Area:
     * If Cube Is Preloaded
        * Sequentially:
        *  Turn 90 degrees (to the right)
        *  Move 1ft
        *  Turn 90 degrees (to the right)
        *  Parallel:
        *    Raise Arm
        *     Move 2ft
        *  Lower Arm (Preloaded Game Piece scored)

        If Cone Is Preloaded
        * Sequentially:
        *  Turn 180 degrees 
        *  Parallel:
        *    Raise Arm
        *     Move 2ft
        *  Lower Arm (Preloaded Game Piece scored)

        AFTER PRELOAD PIECE SCORED:
        Turn 180 Degrees
        Move 10ft
        Turn 90 degrees (to the right)
        Move 4.5ft
        Turn 90 degrees (to the right)
        Move 2.5ft onto Charge Station
        Run SmartBalance
            Smart Balance: Moves backwards and forwards until the gyro
            is within 2.5 degrees level of it's original X Axis rotation

          - - - - - - - - - - 
        * Middle Position From Alliance Area:
     * If Cube Is Preloaded
        * Sequentially:
        *  Turn 180 degrees
        *  Parallel:
        *    Raise Arm
        *     Move 2ft
        *  Lower Arm (Preloaded Game Piece scored)

        If Cone Is Preloaded
        * Sequentially:
        *  Turn 90 degrees 
        *   Move 1ft
        *   Turn 90 Degrees
        *  Parallel:
        *    Raise Arm
        *     Move 2ft
        *  Lower Arm (Preloaded Game Piece scored)

        AFTER PRELOAD PIECE SCORED:
        Turn 180 Degrees
        Move 10ft
        Move 2.5ft backwards onto Charge Station
        Run SmartBalance
            Smart Balance: Moves backwards and forwards until the gyro
            is within 2.5 degrees level of it's original X Axis rotation

     * 
     */
  }

  
}
