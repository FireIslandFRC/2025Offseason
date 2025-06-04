package frc.robot.subsystems.arm;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmIOInputsAutoLogged extends ArmIO.ArmIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("MotorCurrent", motorCurrent);
    table.put("MotorVoltage", motorVoltage);
    table.put("MotorAngle", motorAngle);
  }

  @Override
  public void fromLog(LogTable table) {
    motorCurrent = table.get("MotorCurrent", motorCurrent);
    motorVoltage = table.get("MotorVoltage", motorVoltage);
    motorAngle = table.get("MotorAngle", motorAngle);
  }

  public ArmIOInputsAutoLogged clone() {
    ArmIOInputsAutoLogged copy = new ArmIOInputsAutoLogged();
    copy.motorCurrent = this.motorCurrent;
    copy.motorVoltage = this.motorVoltage;
    copy.motorAngle = this.motorAngle;
    return copy;
  }
}
