import edu.wpilib.wpilibj2.command;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class GroundIntakeSubsystem extends SubsystemBase {
    
    @Override
    // Intake1 is the left intake from the perspective of the bot
    private CANSparkMax intake1;
    @Override
    // Intake2 is the right intake from the perspective of the bot
    private CANSparkMax intake2;

    public GroundIntakeSubsystem(){
        intake1 = new CANSparkMax(1, MotorType.kBrushless);
        intake2 = new CANSparkMax(2, MotorType.kBrushless);
    }
    @Override
    public void periodic() {}

    
    // intake the game piece
    public void intake() {
    }
    
    // set the power of the intake motors
    public void setPower(double power) {
        // TODO: Change the negative and positive after the motors are put on
        intake1.setPower(power);
        intake2.setPower(-power);
    }

    // coasting
    public void coast() {
        intake1.setIdleMode(IdleMode.kCoast);
        intake2.setIdleMode(IdleMode.kCoast);
    }

    // get the current power settings for the motors
    public double[] getPower() {
        // returns the power setting in format intake1, intake2
        return [intake1.get(), intake2.get()];
    }

    // stop the motors
    public void brake() {
        intake1.setIdleMode(IdleMode.kBrake);
        intake2.setIdleMode(IdleMode.kBrake);
    }
}
