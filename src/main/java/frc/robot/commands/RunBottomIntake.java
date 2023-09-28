package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.RunArmWithJoystick;
import frc.robot.subsystems.intake.GroundIntakeSubsystem;
import frc.robot.subsystems.intake.RunGroundIntake;
import frc.robot.subsystems.intake.RunGroundIntakeWithJoystick;
import frc.robot.subsystems.intake.RunGroundOuttake;
import frc.robot.subsystems.wrist.RunWristWithJoystick;
import frc.robot.subsystems.wrist.WristIntake;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristToPosition;


public class RunBottomIntake extends CommandBase {
private WristSubsystem wristSubsystem = new WristSubsystem();
  private GroundIntakeSubsystem groundIntakeSubsystem = new GroundIntakeSubsystem();
  private  RunGroundIntake runGroundIntake = new RunGroundIntake(groundIntakeSubsystem);
  private  RunGroundOuttake runGroundOuttake = new RunGroundOuttake(groundIntakeSubsystem);
  private  WristToPosition wristToPosition = new WristToPosition(wristSubsystem, 5);
 
  public RunBottomIntake(WristSubsystem wristSubsystem, GroundIntakeSubsystem groundIntakesubsystem){
    this.wristSubsystem = wristSubsystem;
    this.groundIntakeSubsystem = groundIntakeSubsystem;
    addRequirements(wristSubsystem);
    addRequirements(groundIntakeSubsystem);
  }
  

  public Command getAutonomousCommand() {
    return new WaitCommand(5);
  }

  public void setDefaultCommands() {
    groundIntakeSubsystem.setDefaultCommand(runGroundIntake);
    new WaitCommand(1);
    wristSubsystem.setDefaultCommand(wristToPosition);
    new WaitCommand(2.5);
    groundIntakeSubsystem.setDefaultCommand(runGroundOuttake);
  
    }
}
