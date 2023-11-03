package frc.robot.subsystems.Arm;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.NetworkTablePath;
import frc.robot.Robot;

public class Arm extends SubsystemBase implements NetworkTablePath {

  private final TalonFX m_leader;
  private final TalonFX m_follower;

  // Feedforward gain NT subscribers
  private final DoubleEntry m_ks = getDoubleEntry("FF/ks", 2);
  private final DoubleEntry m_kg = getDoubleEntry("FF/kg", 2);
  private final DoubleEntry m_kv = getDoubleEntry("FF/kv", 1);
  private final DoubleEntry m_ka = getDoubleEntry("FF/ka", 0);

  // PID gain NT subscribers
  private final DoubleEntry m_kp = getDoubleEntry("PID/kp", 4);
  private final DoubleEntry m_ki = getDoubleEntry("PID/ki", 0.2);
  private final DoubleEntry m_kd = getDoubleEntry("PID/kd", 0.1);

  private final DoubleEntry m_currentAngleEntry = getDoubleEntry("Current Angle", 0);
  private final DoubleEntry m_currentSimAngleEntry = getDoubleEntry("Current Sim Angle", 0);
  private final DoubleEntry m_targetAngleEntry = getDoubleEntry("Target Angle", 0);
  private final DoubleEntry m_currentVoltageEntry = getDoubleEntry("Current Voltage", 0);
  private final DoubleEntry m_currentMotorPosEntry = getDoubleEntry("Current Motor Pos", 0);

  private final Mechanism2d m_armMechanism2d = new Mechanism2d(50, 50);
  private final MechanismLigament2d m_armCurrentAngleLigament2d;
  private final MechanismLigament2d m_armTargetAngleLigament2d;

  private Rotation2d m_targetAngle = Rotation2d.fromDegrees(0);
  private final PositionVoltage positionControl = new PositionVoltage(0)
      .withEnableFOC(true)
      .withSlot(0)
      // .withFeedForward(10)
      .withOverrideBrakeDurNeutral(false)
      .withUpdateFreqHz(100);

  private final Slot0Configs m_gains = new Slot0Configs();

  // Sim Stuff
  private TalonFXSimState m_leaderSim;
  private TalonFXSimState m_followerSim;
  private SingleJointedArmSim m_sim;

  public Arm(int leaderPort, int followerPort) {
    m_leader = new TalonFX(leaderPort, "rio");
    m_follower = new TalonFX(followerPort, "rio");

    m_armCurrentAngleLigament2d = new MechanismLigament2d("Arm", 20, m_targetAngle.getDegrees(), 3,
        new Color8Bit(Color.kGreen));
    m_armTargetAngleLigament2d = new MechanismLigament2d("Arm", 20, 0, 3, new Color8Bit(Color.kBlue));
    m_armMechanism2d.getRoot("Current Arm Root", 5, 10).append(m_armCurrentAngleLigament2d);
    m_armMechanism2d.getRoot("Target Arm Root", 5, 10).append(m_armCurrentAngleLigament2d);
    SmartDashboard.putData("Arm Mechanism", m_armMechanism2d);

    m_gains.GravityType = GravityTypeValue.Arm_Cosine;
    m_gains.kS = m_ks.get();
    m_gains.kG = m_kg.get();
    m_gains.kV = m_kv.get();
    m_gains.kA = m_ka.get();

    m_gains.kP = m_kp.get();
    m_gains.kI = m_ki.get();
    m_gains.kD = m_kd.get();

    MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();

    motorOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;

    m_leader.getConfigurator().apply(m_gains, 0.05);
    m_follower.getConfigurator().apply(m_gains, 0.05);

    m_leader.getConfigurator().apply(motorOutputConfig, 0.05);
    m_follower.getConfigurator().apply(motorOutputConfig, 0.05);

    m_follower.setControl(new Follower(leaderPort, false));

    addSubListener(m_ks, updateGains(ks -> m_gains.kS = ks));
    addSubListener(m_kg, updateGains(kg -> m_gains.kG = kg));
    addSubListener(m_kv, updateGains(kv -> m_gains.kV = kv));
    addSubListener(m_ka, updateGains(ka -> m_gains.kA = ka));

    addSubListener(m_kp, updateGains(kp -> m_gains.kP = kp));
    addSubListener(m_ki, updateGains(ki -> m_gains.kI = ki));
    addSubListener(m_kd, updateGains(kd -> m_gains.kD = kd));

    addSubListener(m_targetAngleEntry, degrees -> m_targetAngle = Rotation2d.fromDegrees(degrees));

    m_targetAngleEntry.accept(20);

    if (Robot.isSimulation()) {
      simulationInit();
    }
  }

  @Override
  public void periodic() {
    m_leader.setControl(positionControl.withPosition(targetAngleToMotor(m_targetAngle.getRadians())));

    m_currentAngleEntry.accept(getAngle().getDegrees());
    m_armCurrentAngleLigament2d.setAngle(getAngle());
    m_armTargetAngleLigament2d.setAngle(m_targetAngle);
  }

  public void setTargetAngle(Rotation2d target) {
    m_targetAngle = target;
    m_targetAngleEntry.accept(m_targetAngle.getDegrees());
  }

  public Rotation2d getAngle() {
    return motorToTargetAngle(m_leader.getPosition().getValueAsDouble());
  }

  public Command setTargetAngleCommand(Rotation2d targetAngle) {
    return runOnce(() -> setTargetAngle(targetAngle));
  }

  public Command setTargetAngleCommand(Supplier<Rotation2d> supplier) {
    return run(() -> setTargetAngle(supplier.get()));
  }

  private double kGearing = 200.0;

  public void simulationInit() {
    double moi = 20.5;
    double armLength = Units.feetToMeters(2);
    double minAngleRad = Units.degreesToRadians(0);
    double maxAngleRad = Units.degreesToRadians(80);
    boolean simulateGravity = true;
    m_sim = new SingleJointedArmSim(
        DCMotor.getFalcon500Foc(2),
        kGearing,
        moi,
        armLength,
        minAngleRad,
        maxAngleRad,
        simulateGravity,
        Units.degreesToRadians(0),
        VecBuilder.fill(0.0001));
    m_leaderSim = m_leader.getSimState();
    m_followerSim = m_follower.getSimState();

    m_leaderSim.Orientation = ChassisReference.CounterClockwise_Positive;
    m_followerSim.Orientation = ChassisReference.CounterClockwise_Positive;
  }

  @Override
  public void simulationPeriodic() {
    m_leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var voltage = m_leaderSim.getMotorVoltage();

    m_currentVoltageEntry.accept(voltage);

    m_sim.setInputVoltage(voltage);
    m_sim.update(0.02);

    m_currentSimAngleEntry.accept(Units.radiansToDegrees(m_sim.getAngleRads()));

    // double batteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.getCurrentDrawAmps());
    // RoboRioSim.setVInVoltage(batteryVoltage);

    m_leaderSim.setRawRotorPosition(targetAngleToMotor(m_sim.getAngleRads()));
    m_leaderSim.setRotorVelocity(targetAngleToMotor(m_sim.getVelocityRadPerSec()));
    m_followerSim.setRawRotorPosition(targetAngleToMotor(m_sim.getAngleRads()));
    m_followerSim.setRotorVelocity(targetAngleToMotor(m_sim.getVelocityRadPerSec()));

    m_currentMotorPosEntry.accept(m_leader.getPosition().getValueAsDouble());
  }

  @Override
  public String getPathRoot() {
    return "Arm";
  }

  private Consumer<Double> updateGains(Consumer<Double> update) {
    return (value) -> {
      update.accept(value);
      m_leader.getConfigurator().apply(m_gains, 0.05);
      m_follower.getConfigurator().apply(m_gains, 0.05);
      System.out.println("Updated arm gains:\n" + m_gains);
    };
  }

  private double targetAngleToMotor(double radians) {
    return Units.radiansToRotations(radians) * kGearing;
  }

  private Rotation2d motorToTargetAngle(double rotations) {
    return Rotation2d.fromRotations(rotations).div(kGearing);
  }
}
