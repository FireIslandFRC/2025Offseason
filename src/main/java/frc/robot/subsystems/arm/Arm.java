package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  // Constructor
  public Arm(ArmIO armIO) {
    this.armIO = armIO;
  }

  // Method to set power for the elevator
  public void setVoltage(double voltage) {
    System.out.println("Elevator position: " + getPosition());
    armIO.set(voltage);
  }

  // Method to stop the elevator
  public void stop() {
    armIO.stop();
  }

  // Set the elevator to a specific position
  public void setPosition(double position) {
    // System.out.println("Elevator position: " + getPosition());
    armIO.setPosition(position);
  }

  // Periodic method called in every cycle (e.g., 20ms)
  @Override
  public void periodic() {
    armIO.updateInputs(inputs);
  }

  public double getPosition() {
    return armIO.getPosition();
  }

  public double getVelocity() {
    return armIO.getVelocity();
  }

  public void resetPosition() {
    armIO.resetPosition();
  }
}