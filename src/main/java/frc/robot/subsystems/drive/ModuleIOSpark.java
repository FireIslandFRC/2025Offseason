// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.backLeftDriveCanId;
import static frc.robot.subsystems.drive.DriveConstants.backLeftTurnCanId;
import static frc.robot.subsystems.drive.DriveConstants.backLeftTurnEncoderCanId;
import static frc.robot.subsystems.drive.DriveConstants.backLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.backRightDriveCanId;
import static frc.robot.subsystems.drive.DriveConstants.backRightTurnCanId;
import static frc.robot.subsystems.drive.DriveConstants.backRightTurnEncoderCanId;
import static frc.robot.subsystems.drive.DriveConstants.backRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.driveEncoderPositionFactor;
import static frc.robot.subsystems.drive.DriveConstants.driveEncoderVelocityFactor;
import static frc.robot.subsystems.drive.DriveConstants.driveKs;
import static frc.robot.subsystems.drive.DriveConstants.driveKv;
import static frc.robot.subsystems.drive.DriveConstants.driveMotorCurrentLimit;
import static frc.robot.subsystems.drive.DriveConstants.frontLeftDriveCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontLeftTurnCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontLeftTurnEncoderCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.frontRightDriveCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontRightTurnCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontRightTurnEncoderCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.turnEncoderInverted;
import static frc.robot.subsystems.drive.DriveConstants.turnEncoderPositionFactor;
import static frc.robot.subsystems.drive.DriveConstants.turnEncoderVelocityFactor;
import static frc.robot.subsystems.drive.DriveConstants.turnInverted;
import static frc.robot.subsystems.drive.DriveConstants.turnMotorCurrentLimit;
import static frc.robot.subsystems.drive.DriveConstants.turnPIDMaxInput;
import static frc.robot.subsystems.drive.DriveConstants.turnPIDMinInput;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, and duty cycle
 * absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO{
    private final Rotation2d zeroRotation;

//     Hardware objects
    private final SparkFlex driveSpark;
    private final SparkFlex turnSpark;
    private final RelativeEncoder driveEncoder;
    private final CANcoder turnEncoder;
    private PIDController rotationPID;
    

    
    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    
    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    

    public ModuleIOSpark(int module) {
        zeroRotation = switch (module) {
            case 0 -> frontLeftZeroRotation;
            case 1 -> frontRightZeroRotation;
            case 2 -> backLeftZeroRotation;
            case 3 -> backRightZeroRotation;
            default -> new Rotation2d();};
        driveSpark = new SparkFlex(
                switch (module) {
                    case 0 -> frontLeftDriveCanId;
                    case 1 -> frontRightDriveCanId;
                    case 2 -> backLeftDriveCanId;
                    case 3 -> backRightDriveCanId;
                    default -> 0;
                },
                MotorType.kBrushless);
        turnSpark = new SparkFlex(
                switch (module) {
                    case 0 -> frontLeftTurnCanId;
                    case 1 -> frontRightTurnCanId;
                    case 2 -> backLeftTurnCanId;
                    case 3 -> backRightTurnCanId;
                    default -> 0;
                },
                MotorType.kBrushless);
        driveEncoder = driveSpark.getEncoder();
        turnEncoder = new CANcoder(
                switch (module) {
                        case 0 -> frontLeftTurnEncoderCanId;
                        case 1 -> frontRightTurnEncoderCanId;
                        case 2 -> backLeftTurnEncoderCanId;
                        case 3 -> backRightTurnEncoderCanId;
                        default -> 21;
                }
        );
        rotationPID = new PIDController(
                DriveConstants.KP_TURNING,
                DriveConstants.KI_TURNING,
                DriveConstants.KD_TURNING);
        // Configure drive motor
        var driveConfig = new SparkFlexConfig();
        driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(driveMotorCurrentLimit)
                .voltageCompensation(12.0);
        driveConfig
                .encoder
                .positionConversionFactor(driveEncoderPositionFactor)
                .velocityConversionFactor(driveEncoderVelocityFactor);
        tryUntilOk(
                driveSpark,
                5,
                () -> driveSpark.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(turnInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(turnMotorCurrentLimit)
                .voltageCompensation(12.0);
        turnConfig
                .absoluteEncoder
                .inverted(turnEncoderInverted)
                .positionConversionFactor(turnEncoderPositionFactor)
                .velocityConversionFactor(turnEncoderVelocityFactor);
        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, () -> turnEncoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        sparkStickyFault = false;

        ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
        ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);

        ifOk(
                driveSpark,
                new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
                (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        ifOk(
                turnSpark,
                () -> turnEncoder.getAbsolutePosition().getValueAsDouble(),
                (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnSpark, () -> turnEncoder.getVelocity().getValueAsDouble(), (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
                turnSpark,
                new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
                drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
        driveSpark.set(ffVolts);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint =
                MathUtil.inputModulus(rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
        turnSpark.set(rotationPID.calculate(setpoint));
    }
}
