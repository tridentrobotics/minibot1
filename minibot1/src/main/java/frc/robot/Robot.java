package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Robot extends TimedRobot {
  // Left Swerve Module
  private TalonFX leftDriveMotor;
  private TalonFX leftTurnMotor;
  private CANcoder leftEncoder;
  
  // Right Swerve Module
  private TalonFX rightDriveMotor;
  private TalonFX rightTurnMotor;
  private CANcoder rightEncoder;
  
  // Gyro
  private Pigeon2 gyro;
  
  // Controller
  private XboxController controller;
  
  // CAN IDs - Based on your Phoenix Tuner
  private static final int LEFT_DRIVE_ID = 2;
  private static final int LEFT_TURN_ID = 4;
  private static final int LEFT_ENCODER_ID = 0;
  
  private static final int RIGHT_DRIVE_ID = 3;
  private static final int RIGHT_TURN_ID = 5;
  private static final int RIGHT_ENCODER_ID = 1;
  
  private static final int GYRO_ID = 0;
  
  // Speed constants
  private static final double MAX_SPEED = 0.5;  // 50% for safety
  
  @Override
  public void robotInit() {
    // Initialize left module
    leftDriveMotor = new TalonFX(LEFT_DRIVE_ID);
    leftTurnMotor = new TalonFX(LEFT_TURN_ID);
    leftEncoder = new CANcoder(LEFT_ENCODER_ID);
    
    // Initialize right module
    rightDriveMotor = new TalonFX(RIGHT_DRIVE_ID);
    rightTurnMotor = new TalonFX(RIGHT_TURN_ID);
    rightEncoder = new CANcoder(RIGHT_ENCODER_ID);
    
    // Initialize gyro
    gyro = new Pigeon2(GYRO_ID);
    
    // Initialize controller
    controller = new XboxController(0);
    
    // Configure all motors to brake mode
    leftDriveMotor.setNeutralMode(NeutralModeValue.Brake);
    leftTurnMotor.setNeutralMode(NeutralModeValue.Brake);
    rightDriveMotor.setNeutralMode(NeutralModeValue.Brake);
    rightTurnMotor.setNeutralMode(NeutralModeValue.Brake);
    
    System.out.println("Swerve Minibot Initialized!");
  }
  
  @Override
  public void teleopInit() {
    gyro.reset();
    System.out.println("Teleop started - Gyro reset!");
  }
  
  @Override
  public void teleopPeriodic() {
    // Get controller inputs
    double forward = -controller.getLeftY();  // Forward/backward
    double strafe = controller.getLeftX();    // Left/right
    double rotation = controller.getRightX(); // Rotation
    
    // Apply deadband
    forward = applyDeadband(forward, 0.1);
    strafe = applyDeadband(strafe, 0.1);
    rotation = applyDeadband(rotation, 0.1);
    
    // For a 2-wheel swerve (simplified differential swerve)
    // Calculate desired speeds for each module
    double leftSpeed = (forward - rotation) * MAX_SPEED;
    double rightSpeed = (forward + rotation) * MAX_SPEED;
    
    // Set drive motor speeds
    leftDriveMotor.set(leftSpeed);
    rightDriveMotor.set(rightSpeed);
    
    // For now, keep turn motors at 0 (straight forward)
    // You'll need proper swerve kinematics for full functionality
    leftTurnMotor.set(0);
    rightTurnMotor.set(0);
    
    // Reset gyro with A button
    if (controller.getAButtonPressed()) {
      gyro.reset();
      System.out.println("Gyro reset!");
    }
    
    // Print diagnostics with B button
    if (controller.getBButtonPressed()) {
      System.out.println("Gyro: " + gyro.getYaw().getValueAsDouble());
      System.out.println("Left Encoder: " + leftEncoder.getAbsolutePosition().getValueAsDouble());
      System.out.println("Right Encoder: " + rightEncoder.getAbsolutePosition().getValueAsDouble());
    }
  }
  
  @Override
  public void disabledInit() {
    // Stop all motors
    leftDriveMotor.set(0);
    rightDriveMotor.set(0);
    leftTurnMotor.set(0);
    rightTurnMotor.set(0);
  }
  
  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    return value;
  }
}
