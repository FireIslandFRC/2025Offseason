package frc.robot.subsystems.arm;
 
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class ArmIOSparkFlex implements ArmIO {
  private final SparkFlex armMotor;
  private final RelativeEncoder encoder;
  private final SparkFlexConfig armMotorConfig;
  private final SparkClosedLoopController armCLController;

  // Constructor
  public ArmIOSparkFlex() {
    // Initialize the CANSparkMax motors for main and follower
    armMotor = new SparkFlex(10, MotorType.kBrushless); //FIXME: motor id



    armMotorConfig = new SparkFlexConfig();

                

    armMotorConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40)
                    .inverted(true);
    armMotorConfig.encoder
                    .positionConversionFactor(1) // meters   CHECKME make sure right conversion
                    .velocityConversionFactor(1/60); // meters per second
    armMotorConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(1,0,0);

    encoder = armMotor.getEncoder();
    armCLController = armMotor.getClosedLoopController();

  }

  @Override
  public void set(double voltage) {
    // Set the power to the main motor
    armMotor.set(voltage);
  }

  @Override
  public double getPosition() {
    // Get the position from the encoder
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    // Get the velocity from the encoder
    return encoder.getVelocity();
  }

  @Override
  public void resetPosition() {
    // Reset the encoder to the specified position
    encoder.setPosition(0);
  }

  @Override
  public void setPosition(double position) {
    armCLController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void stop() {
    armMotor.setVoltage(0);
  }
}