package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PowerConstants;
import frc.robot.subsystems.GroundIntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;


public class RunGroundIntakeWithJoystick extends CommandBase {
    private GroundIntakeSubsystem groundIntakeSubsystem;
    private XboxController joystick;
    
    public RunGroundIntakeWithJoystick(GroundIntakeSubsystem groundIntakeSubsystem, XboxController joystick) {
        this.groundIntakeSubsystem = groundIntakeSubsystem;
        this.joystick = joystick;
        addRequirements(groundIntakeSubsystem);
    }
    
    @Override
    public void initialize(){
        groundIntakeSubsystem.setAllPower(0);
    }


    @Override
    public void execute() {
        if(joystick.getRightTriggerAxis() > 0.05){
            groundIntakeSubsystem.runIntake(PowerConstants.groundIntakePower);
        }
        else if(joystick.getLeftTriggerAxis() > 0.05){
            groundIntakeSubsystem.runIntake(-PowerConstants.groundIntakePower);
        }
        else{
            groundIntakeSubsystem.setAllPower(0);
            groundIntakeSubsystem.brake();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}