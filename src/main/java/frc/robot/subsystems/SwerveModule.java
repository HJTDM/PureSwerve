// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  private PearadoxSparkMax driveMotor;
  private PearadoxSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private PIDController turnPIDController;
  private CANcoder absoluteEncoder;

  private double absoluteEncoderOffset;
  private double lastAngle;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset) {
      this.absoluteEncoderOffset = absoluteEncoderOffset;

      absoluteEncoder = new CANcoder(absoluteEncoderId);

      driveMotor = new PearadoxSparkMax(driveMotorId, MotorType.kBrushless, IdleMode.kCoast, 45, driveMotorReversed);
      turnMotor = new PearadoxSparkMax(turnMotorId, MotorType.kBrushless, IdleMode.kCoast, 25, turnMotorReversed);

      driveEncoder = driveMotor.getEncoder();
      turnEncoder = turnMotor.getEncoder();

      turnPIDController = new PIDController(SwerveConstants.KP_TURNING, 0, 0);
      turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

      resetEncoders();
      lastAngle = getTurnMotorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getDriveMotorPosition(){
    return driveEncoder.getPosition() * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
  }

  public double getDriveMotorVelocity(){
    return driveEncoder.getVelocity() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
  }

  public double getTurnMotorPosition(){
    return turnEncoder.getPosition() * SwerveConstants.TURN_MOTOR_PCONVERSION;
  }

  public double getTurnMotorVelocity(){
    return turnEncoder.getPosition() * SwerveConstants.TURN_MOTOR_VCONVERSION;
  }

  public double getAbsoluteEncoderAngle(){
    double angle = absoluteEncoder.getAbsolutePosition().getValue();
    angle -= absoluteEncoderOffset;
    angle *= (Math.PI / 180);
    return angle;
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
    turnEncoder.setPosition(getAbsoluteEncoderAngle() / SwerveConstants.TURN_MOTOR_PCONVERSION);
  }

  public void setDesiredState(double speed, double angle){
    double delta = angle - lastAngle;
    if (Math.abs(delta) > 90.0) {
      speed = -speed;
      angle -= 180;

      if(angle <= -180){
        angle += 360;
      }
    }

    angle = (Math.abs(speed) <= (SwerveConstants.DRIVETRAIN_MAX_SPEED * 0.01)) ? lastAngle : angle;

    setAngle(angle);
    setSpeed(speed);
  }

  public void setSpeed(double speed){
    driveMotor.set(speed / SwerveConstants.DRIVETRAIN_MAX_SPEED);
  }

  public void setAngle(double angle){
    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), angle));
    lastAngle = angle;
  }

  public void stop(){
    driveMotor.set(0);
    turnMotor.set(0);
  }

  public void setBrake(boolean brake){
    if(brake){
      driveMotor.setIdleMode(IdleMode.kBrake);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
    else{
      driveMotor.setIdleMode(IdleMode.kCoast);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
  }
}
