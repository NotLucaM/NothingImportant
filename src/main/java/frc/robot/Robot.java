/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Joystick joystick1 = new Joystick(0);
  Joystick joystick2 = new Joystick(1);

  CANSparkMax leftSpark1 = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushed);
  CANSparkMax leftSpark2 = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushed);
  CANSparkMax leftSpark3 = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushed);

  CANSparkMax rightSpark1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushed);
  CANSparkMax rightSpark2 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushed);
  CANSparkMax rightSpark3 = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushed);

  SpeedControllerGroup speedControllerLeft = new SpeedControllerGroup(leftSpark1, leftSpark2, leftSpark3);
  SpeedControllerGroup speedControllerRight = new SpeedControllerGroup(rightSpark1, rightSpark2, rightSpark3);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double speed = pid();

    speedControllerLeft.set(speed);
    speedControllerRight.set(-speed);
  }

  public double getLoc() { // 18 * pi/23
      return conversionFactor(leftSpark1.getEncoder().getPosition());
  }

  public double conversionFactor(double in) {
      return in  * Math.PI * 18 / 23;
  }

  double target = 60;
  double lastError = target;
  double integral = 0;

  public double pid() {
      double kP = 0.01;
      double kD = 0;
      double kI = 0;

      double error = target - getLoc();
      double v = (error - lastError) / 0.02;
      integral += error * 0.02;

      lastError = error;
      return 0.3 * (kP * error + kD * v + kI * integral);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double dirX = joystick1.getX() * 0.3;
    double dirY = joystick2.getY() * 0.3;

    speedControllerLeft.set(dirX - dirY);
    speedControllerRight.set(-(dirX + dirY));
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
