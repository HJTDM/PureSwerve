// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {
  private SwerveModule leftFront = new SwerveModule(
    SwerveConstants.LEFT_FRONT_DRIVE_ID, 
    SwerveConstants.LEFT_FRONT_TURN_ID, 
    false, 
    true, 
    SwerveConstants.LEFT_FRONT_CANCODER_ID, 
    SwerveConstants.LEFT_FRONT_OFFSET);

  private SwerveModule rightFront = new SwerveModule(
    SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
    SwerveConstants.RIGHT_FRONT_TURN_ID, 
    false, 
    true, 
    SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
    SwerveConstants.RIGHT_FRONT_OFFSET);

  private SwerveModule leftBack = new SwerveModule(
    SwerveConstants.LEFT_BACK_DRIVE_ID, 
    SwerveConstants.LEFT_BACK_TURN_ID, 
    false, 
    true, 
    SwerveConstants.LEFT_BACK_CANCODER_ID, 
    SwerveConstants.LEFT_BACK_OFFSET);

  private SwerveModule rightBack = new SwerveModule(
    SwerveConstants.RIGHT_BACK_DRIVE_ID, 
    SwerveConstants.RIGHT_BACK_TURN_ID, 
    false, 
    true, 
    SwerveConstants.RIGHT_BACK_CANCODER_ID, 
    SwerveConstants.RIGHT_BACK_OFFSET);

  private SlewRateLimiter frontLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter sideLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter turnLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

  private double[] wheelSpeeds = new double[4];
  private double[] wheelAngles = new double[4];

  private Pigeon2 gyro = new Pigeon2(SwerveConstants.PIGEON_ID);

  private static final Drivetrain DRIVETRAIN = new Drivetrain();

  public static Drivetrain getInstance(){
    return DRIVETRAIN;
  }

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void swerveDrive(double xSpeed, double ySpeed, double omegaSpeed, boolean fieldRelative, boolean deadband){
    if(deadband){
      xSpeed = Math.abs(xSpeed) > 0.1 ? xSpeed : 0;
      ySpeed = Math.abs(ySpeed) > 0.1 ? ySpeed : 0;
      omegaSpeed = Math.abs(omegaSpeed) > 0.1 ? omegaSpeed : 0;
    }

    xSpeed = frontLimiter.calculate(xSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    ySpeed = sideLimiter.calculate(ySpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    omegaSpeed = turnLimiter.calculate(omegaSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    if(fieldRelative){
      double robotAngle = getHeading();

      xSpeed = ySpeed * Math.sin(robotAngle) + xSpeed * Math.cos(robotAngle);
      ySpeed = ySpeed * Math.cos(robotAngle) - xSpeed * Math.sin(robotAngle);
    }

    double A = xSpeed - omegaSpeed * (SwerveConstants.WHEEL_BASE / 2);
    double B = xSpeed + omegaSpeed * (SwerveConstants.WHEEL_BASE / 2);
    double C = ySpeed - omegaSpeed * (SwerveConstants.TRACK_WIDTH / 2);
    double D = ySpeed + omegaSpeed * (SwerveConstants.TRACK_WIDTH / 2);

    calculateModuleStates(A, B, C, D);
    normalizeWheelSpeeds();
    setModuleStates();
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public void setHeading(double heading){
    gyro.setYaw(heading);
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw().getValue(), 360);
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }

  public void resetAllEncoders(){
    leftFront.resetEncoders();
    rightFront.resetEncoders();
    leftBack.resetEncoders();
    rightBack.resetEncoders();
  }

  public void setAllIdleMode(boolean brake){
    if(brake){
      leftFront.setBrake(true);
      rightFront.setBrake(true);
      leftBack.setBrake(true);
      rightBack.setBrake(true);
    }
    else{
      leftFront.setBrake(false);
      rightFront.setBrake(false);
      leftBack.setBrake(false);
      rightBack.setBrake(false);
    }
  }

  public void setModuleStates(){
    leftFront.setDesiredState(wheelSpeeds[0], wheelAngles[0]);
    rightFront.setDesiredState(wheelSpeeds[1], wheelAngles[1]);
    leftBack.setDesiredState(wheelSpeeds[2], wheelAngles[2]);
    rightBack.setDesiredState(wheelSpeeds[3], wheelAngles[3]);
  }

  public void calculateModuleStates(double A, double B, double C, double D){
    wheelSpeeds[0] = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));
    wheelSpeeds[1] = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
    wheelSpeeds[2] = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
    wheelSpeeds[3] = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));

    wheelAngles[0] = C == 0 && A == 0 ? 0 : Math.atan2(C, A);
    wheelAngles[0] = D == 0 && A == 0 ? 0 : Math.atan2(D, A);
    wheelAngles[0] = C == 0 && B == 0 ? 0 : Math.atan2(C, B);
    wheelAngles[0] = D == 0 && B == 0 ? 0 : Math.atan2(D, B);
  }

  private void normalizeWheelSpeeds(){
    double maxModuleSpeed = Arrays.stream(wheelSpeeds).max().getAsDouble();

    if (maxModuleSpeed > SwerveConstants.DRIVETRAIN_MAX_SPEED) {
      for (double wheelSpeed : wheelSpeeds) {
        wheelSpeed = wheelSpeed / maxModuleSpeed * SwerveConstants.DRIVETRAIN_MAX_SPEED;
      }
    }
  }
}
