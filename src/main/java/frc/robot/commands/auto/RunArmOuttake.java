package frc.robot.commands.auto;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class RunArmOuttake extends CommandBase{
    IntakeSubsystem intakeSubsystem;
    public RunArmOuttake(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.coast();
        intakeSubsystem.setPower(-5);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.coast();
        intakeSubsystem.brake();
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}