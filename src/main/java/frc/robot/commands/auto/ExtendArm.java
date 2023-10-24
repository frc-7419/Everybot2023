package frc.robot.commands.auto;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmToPosition;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ExtendArm extends CommandBase{
    ArmSubsystem armSubsystem;
    ArmToPosition armToPosition;

    public ExtendArm(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // fix the setpoint later
        armToPosition = new ArmToPosition(armSubsystem, 5);
        armToPosition = new ArmToPosition(armSubsystem, 0);
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