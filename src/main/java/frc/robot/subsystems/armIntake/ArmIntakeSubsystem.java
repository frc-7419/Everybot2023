// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.armIntake;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import frc.robot.Constants;

// public class ArmIntakeSubsystem extends SubsystemBase {
//   /** Creates a new armIntakeSubsystem. */
//   private CANSparkMax armMotor;
//   public ArmIntakeSubsystem() {
//     armMotor = new CANSparkMax(Constants.CanIds.armIntake.id, MotorType.kBrushless);
//     armMotor.setInverted(true);
//   }

//   public void setVoltage(double power) {
//     armMotor.setVoltage(power);
//   }
//   public void setSpeed(double speed){
//     armMotor.set(speed);
//   }

//   public void coast(){
//     armMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
//   }
//   public void brake(){
//     armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }

