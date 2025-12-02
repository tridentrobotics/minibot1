package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Robot extends TimedRobot {
  // Motors - Update these CAN IDs to match your setup
  private TalonFX leftMotor;   // Kraken connected to left wheel
  private TalonFX rightMotor;  // Kraken connected to right wheel
  
  // Gyro
  private Pigeon2 gyro;
  
  // Controller
  private XboxController controller;
  
  // Drive system
  private DifferentialDrive drive;
  
  // Constants
  private static final int LEFT_MOTOR_ID = 1;    // Change to your left motor CAN ID
  private static final int RIGHT_MOTOR_ID = 2;   // Change to your right motor CAN ID
  private static final int GYRO_ID = 3;          // Change to your gyro CAN ID
  
  // Speed limiters (0.0 to 1.0)
  private static final double MAX_SPEED = 0.7;     // 70% max speed for safety
  private static final double TURN_SPEED = 0.6;    // 60% turn speed
  
  @Override
  public void robotInit() {
    // Initialize motors
    leftMotor = new TalonFX(LEFT_MOTOR_ID);
    rightMotor = new TalonFX(RIGHT_MOTOR_ID);
    
    // Initialize gyro
    gyro = new Pigeon2(GYRO_ID);
    
    // Initialize controller
    controller = new XboxController(0);
    
    // Configure motors
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
    
    // Invert right motor so both spin same direction for forward
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotor.getConfigurator().apply(rightConfig);
    
    // Create differential drive
    drive = new DifferentialDrive(leftMotor::set, rightMotor::set);
    drive.setMaxOutput(MAX_SPEED);
    
    System.out.println("Minibot initialized!");
  }
  
  @Override
  public void teleopInit() {
    // Reset gyro when teleop starts
    gyro.reset();
  }
  
  @Override
  public void teleopPeriodic() {
    // Arcade drive: Left stick Y for forward/back, Right stick X for turning
    double speed = -controller.getLeftY() * MAX_SPEED;     // Forward/backward
    double turn = controller.getRightX() * TURN_SPEED;     // Left/right turn
    
    // Apply deadband to avoid stick drift
    speed = applyDeadband(speed, 0.1);
    turn = applyDeadband(turn, 0.1);
    
    // Drive the robot
    drive.arcadeDrive(speed, turn);
    
    // Optional: Reset gyro with A button
    if (controller.getAButtonPressed()) {
      gyro.reset();
      System.out.println("Gyro reset!");
    }
    
    // Optional: Print gyro angle with B button
    if (controller.getBButtonPressed()) {
      System.out.println("Gyro angle: " + gyro.getYaw().getValueAsDouble());
    }
  }
  
  @Override
  public void disabledInit() {
    // Stop motors when disabled
    drive.stopMotor();
  }
  
  // Helper method to eliminate stick drift
  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    return value;
  }
}