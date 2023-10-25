// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import static frc.robot.Constants.PIDConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmWithPIDCommand extends PIDCommand {
  /** Creates a new ArmWithPID. */
  private double tolerance = 0.5;
  public ArmWithPIDCommand(ArmSubsystem armSubsystem, double setpoint) {
    super(
      // The controller that the command will use
      new PIDController(ArmAngleKp, ArmAngleKi, ArmAngleKd),

      // This should return the measurement
      () -> armSubsystem.getPosition() - Constants.RobotConstants.armEncoderOffset,

      // This should return the setpoint (can also be a constant)
      ()-> setpoint,

      // This uses the output
      output -> {
        SmartDashboard.putNumber("Arm Output", output);
        armSubsystem.setPower(output);
      });
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(armSubsystem);

      // Configure additional PID options by calling `getController` here.
      getController().setTolerance(tolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
