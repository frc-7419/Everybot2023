package frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.GroundIntakeSubsystem;
import frc.robot.Constants;

public class RunGroundIntake extends CommandBase {
    private GroundIntakeSubsystem groundIntakeSubsystem
    private XboxController joystick;

    @Override
    public runIntakeWithJoystick(GroundIntakeSubsystem groundIntakeSubsystem, XboxController joystick) {
        this.groundIntakeSubsystem = groundIntakeSubsystem;
        this.joystick = joystick;
        
    }
    
    @Override
    public void initialize(){
        groundIntakeSubsystem.setPower(0)
    }

    @Override
    public void runIntake(double power) {
        groundIntakeSubsystem.coast();
        groundIntakeSubsystem.setPower(power);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Intake Power", intakeSubsystem.getPower());
        if (joystick.getLeftTriggerAxis > 0) {
            runIntake(Constants.IntakePower);
        }
        else if (joystick.getRightTriggerAxis > 0) {
            runIntake(-Constants.IntakePower);

        }
        else {
            groundIntakeSubsystem.setPower(0);
            groundIntakeSubsystem.brake();
        }
    }

    @Override
    public boolean isFinished() {
        return False;
    }

    
}