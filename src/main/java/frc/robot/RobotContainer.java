// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Map;
import java.util.Optional;
import java.util.Set;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private static final int kShooterMotorCanId = 50; // Kraken X60 (Talon FX)
    private static final String kShooterCanBus = "rio";
    private static final int kFeederMotorCanId = 20;
    private static final int kIntakeMotorCanId = 21; // NEO brushless
    private static final int kIntakeArmMotorCanId = 22; // NEO brushless

    private static final double kIntakePercentOutput = 0.8;
    private static final double kFeederPercentOutput = -0.6;
    private static final double kFeederReversePercentOutput = 0.6;
    private static final double kShooterPercentOutput = 0.6;
    private static final double kShooterBoostPercentOutput = 0.8; // Hold D-pad Right while shooter is on
    private static final double kShooterMaxVelocityRps = 80.0; // tune: max useful flywheel speed (rotations/sec)
    private static final double kShooterKs = 0.15;  // volts to overcome static friction
    private static final double kShooterKv = 0.12;  // volts per RPS (~12V / 100 RPS for Kraken)
    private static final double kShooterKp = 0.3;   // volts per RPS of error
    private static final double kShooterAtSpeedThresholdRps = 3.0;
    private static final double kIntakeArmUpPercentOutput = -0.3;   // counterclockwise
    private static final double kIntakeArmDownPercentOutput = 0.3;  // clockwise
    private static final double kIntakeArmMinRotations = -2.0;     // soft limit (tune on robot)
    private static final double kIntakeArmMaxRotations = 2.0;      // soft limit (tune on robot)
    private static final double kIntakeArmStallCurrentAmps = 35.0; // current above this = stalled
    private static final double kIntakeStallCurrentAmps = 40.0;    // intake roller stall protection
    private static final double kIntakeDiagPeriodSec = 1.0;
    private static final String kLimelightTableName = "limelight-back";
    private static final String kLimelightStreamUrlDefault = "http://limelight-back.local:5800/stream.mjpeg";
    private static final double kLimelightTurnKp = 2.0; // rad/s per rad of tx error
    private static final double kLimelightMaxTurnRateRadPerSec = 1.5;
    private static final double kLimelightTxDeadbandDeg = 1.0;
    private static final double kInputDeadband = 0.10;
    private static final double kOperatorDriveSpeedScale = 0.2; // Editable operator drive/turn speed scale
    private static final double kOperatorOverrideOnThreshold = 0.20;
    private static final double kOperatorOverrideOffThreshold = 0.08;
    private static final double kDriveSlewRateMpsPerSec = 6.0;
    private static final double kTurnSlewRateRadPerSec2 = 16.0;
    private static final double kIdleLinearSpeedThresholdMps = 0.03;
    private static final double kIdleTurnRateThresholdRadPerSec = 0.05;
    private static final boolean kEnableIntakeArmPositionDebug = false;
    private static final double kIntakeArmPosLogPeriodSec = 0.5;
    private static final String kCenterAutoName = "Left_trench";
    private static final double kAutoShooterSpinupSec = 1.5;
    private static final double kAutoFeedSec = 5.0;

    // Shift-aware scoring system
    private static final double kRumblePreActiveSec = 3.0;
    private static final double kRumbleIntensity = 0.8;
    private static final double kEstimatedFuelPerSec = 2.0;
    private static final double kAutoFeedAimThresholdDeg = 5.0;

    // Auto-cycle teleop (button-driven autonomous scoring)
    // Depot positions derived from alliance wall AprilTag pairs (tags 29/30 blue, 13/14 red).
    // Robot parks ~1.2m from the wall, centered on the depot opening.
    private static final Translation2d kBlueDepotPosition = new Translation2d(1.2, 0.88);
    private static final Translation2d kRedDepotPosition = new Translation2d(15.34, 7.19);
    private static final double kPathfindMaxVelMps = 3.0;
    private static final double kPathfindMaxAccelMps2 = 3.0;
    private static final double kAutoIntakeMaxSec = 4.0;
    private static final double kAutoScoreMaxSec = 12.0;
    private static final double kManualOverrideThreshold = 0.3;

    // Smart cycle: current-based detection for hopper empty and collection done
    private static final double kFeederEmptyCurrentAmps = 4.0;
    private static final double kFeederEmptyConfirmSec = 0.4;
    private static final double kMinScoringSec = 2.0;
    private static final double kIntakeIdleCurrentAmps = 5.0;
    private static final double kIntakeIdleConfirmSec = 0.5;
    private static final double kMinCollectSec = 1.0;
    private static final double kVisionFreshnessTimeoutSec = 2.0;
    private static final double kAutoCycleTimeoutSec = 120.0;
    private static final double kFieldMarginMeters = 0.5;

    // 2026 REBUILT field dimensions (blue-origin WPILib coordinates, meters)
    private static final double kFieldLengthMeters = 16.541;
    private static final double kFieldWidthMeters = 8.069;
    // Hub centers derived from AprilTag positions (average of all 8 face tags per hub).
    // The manual's "4.03m" is the BACK FACE distance; the actual center is ~4.63m from the wall.
    private static final Translation2d kBlueHubPosition = new Translation2d(4.63, 4.03);
    private static final Translation2d kRedHubPosition = new Translation2d(11.92, 4.03);

    // Distance-based shooter speed: dutyCycle = kShooterDistA + kShooterDistB * sqrt(distance)
    private static final double kShooterDistA = 0.35;        // base offset
    private static final double kShooterDistB = 0.18;        // sqrt scaling coefficient
    private static final double kShooterDistMinOutput = 0.30; // floor clamp
    private static final double kShooterDistMaxOutput = 1.0;  // ceiling clamp
    private static final double kVisionPoseLogPeriodSec = 0.5;

    // Auto-aim-to-hub P-controller
    private static final double kAimToHubKp = 3.0;                    // rad/s per rad of heading error
    private static final double kAimToHubMaxTurnRateRadPerSec = 2.0;  // clamp
    private static final double kAimToHubDeadbandDeg = 2.0;           // stop turning within this
    private static final double kShooterFacingOffsetDeg = 0.0;        // shooter at rear but balls arc forward

    // 3D targeting: auto-drive to ideal shooting position (active during Y hold)
    private static final double kIdealShootingDistanceMeters = 2.5;   // tune: sweet spot range to hub
    private static final double kRangeDistDeadbandMeters = 0.15;      // close enough to ideal range
    private static final double kRangeKp = 1.5;                       // m/s per meter of range error
    private static final double kMaxRangeSpeedMps = 1.5;              // max approach/retreat speed
    private static final double kStrafeKp = 2.0;                      // m/s per meter of lateral offset
    private static final double kMaxStrafeSpeedMps = 1.0;             // max strafe speed
    private static final double kStrafeDeadbandMeters = 0.10;         // close enough laterally

    // 2026 REBUILT hub AprilTag IDs (8 tags per hub, 2 per face on all 4 faces)
    private static final int[] kBlueHubTagIds = {2, 3, 4, 5, 8, 9, 10, 11};
    private static final int[] kRedHubTagIds  = {18, 19, 20, 21, 24, 25, 26, 27};

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Closed-loop velocity helps reduce current spikes/jitter
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final TalonFX shooterMotor = new TalonFX(kShooterMotorCanId, kShooterCanBus);
    private final SparkMax feederMotor = new SparkMax(kFeederMotorCanId, MotorType.kBrushed);
    private final SparkMax intakeMotor = new SparkMax(kIntakeMotorCanId, MotorType.kBrushless);
    private final SparkMax intakeArmMotor = new SparkMax(kIntakeArmMotorCanId, MotorType.kBrushless);
    private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut shooterNeutralRequest = new NeutralOut();
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(kLimelightTableName);
    private double lastIntakeDiagTimeSec = -1.0;
    private double lastIntakeArmPosLogTimeSec = -1.0;
    private double shooterCommandedRps = 0.0;
    private boolean limelightAlignModeLast = false;
    private boolean limelightHadTagLast = false;
    private double lastLimelightDebugTimeSec = -1.0;
    private boolean operatorOverrideLatched = false;
    private final SlewRateLimiter xVelLimiter = new SlewRateLimiter(kDriveSlewRateMpsPerSec);
    private final SlewRateLimiter yVelLimiter = new SlewRateLimiter(kDriveSlewRateMpsPerSec);
    private final SlewRateLimiter rotVelLimiter = new SlewRateLimiter(kTurnSlewRateRadPerSec2);

    private double lastVisionPoseLogTimeSec = -1.0;
    private double lastDistanceToHubMeters = -1.0;
    private boolean hadVisionPose = false;
    private double lastVisionTagTimeSec = -1.0;
    private boolean aimToHubModeLast = false;

    private int estimatedFuelScored = 0;
    private double fuelAccumulator = 0.0;
    private double lastShiftUpdateSec = -1.0;

    private final PathConstraints pathfindConstraints = new PathConstraints(
        kPathfindMaxVelMps, kPathfindMaxAccelMps2, 2 * Math.PI, 4 * Math.PI);
    private boolean autoCycleActive = false;
    private double feederLowCurrentStartSec = -1.0;
    private double intakeIdleStartSec = -1.0;
    private double scoringPhaseStartSec = -1.0;
    private double collectPhaseStartSec = -1.0;

    private final Field2d field2d = new Field2d();
    private VideoSource limelightCamera;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureShooter();
        configureBindings();
        configureLimelightStream();
        publishTunableDefaults();
        configureDashboard();
        DriverStation.reportWarning(
            "[AUTO] PathPlanner configured: " + drivetrain.isAutoBuilderConfigured(),
            false
        );
    }

    private void publishTunableDefaults() {
        SmartDashboard.putNumber("Tune/Ideal Distance (m)", kIdealShootingDistanceMeters);
        SmartDashboard.putNumber("Tune/Shooter A", kShooterDistA);
        SmartDashboard.putNumber("Tune/Shooter B", kShooterDistB);
        SmartDashboard.putNumber("Tune/Shooter Max RPS", kShooterMaxVelocityRps);
        SmartDashboard.putNumber("Tune/Shooter Min %", kShooterDistMinOutput);
        SmartDashboard.putNumber("Tune/Shooter Max %", kShooterDistMaxOutput);
        SmartDashboard.putNumber("Tune/Aim Kp", kAimToHubKp);
        SmartDashboard.putNumber("Tune/Range Kp", kRangeKp);

        SmartDashboard.putString("Limelight Stream URL", kLimelightStreamUrlDefault);

        SmartDashboard.putString("Tune/Step 1 - First Boot",
            "Power on robot. Open SmartDashboard or Shuffleboard. "
            + "Check that 'Vision Valid' shows TRUE (Limelight sees AprilTags). "
            + "Check that 'Hub Distance (m)' shows a reasonable number. "
            + "and camera offsets are configured in http://limelight-back.local:5801. "
            + "If Vision feed is blank: set 'Limelight Stream URL' to http://10.0.0.1:5800/stream.mjpeg (or Limelight's IP), restart robot.");

        SmartDashboard.putString("Tune/Step 2 - Test Auto-Aim",
            "Face the robot toward the hub. Hold Y button. "
            + "The robot should rotate to point its front at the hub. "
            + "Watch 'Aim Error (deg)' -- it should drop toward 0. "
            + "If the robot WOBBLES back and forth: lower 'Aim Kp' (try 2.0, then 1.5). "
            + "If it turns TOO SLOWLY: raise 'Aim Kp' (try 4.0, then 5.0). "
            + "When 'Aimed' shows TRUE, the robot is locked on.");

        SmartDashboard.putString("Tune/Step 3 - Test Auto-Drive",
            "Hold Y button from ~5 meters away. The robot will auto-drive toward the hub "
            + "and stop at 'Ideal Distance' (default 2.5m). "
            + "Watch '3D Target Range Err (m)' -- it should go to 0. "
            + "Watch '3D Target In Range' -- it should turn TRUE when parked. "
            + "If the robot OVERSHOOTS and goes back and forth: lower 'Range Kp' (try 1.0, then 0.7). "
            + "While holding Y, your left stick lets you orbit/strafe around the hub.");

        SmartDashboard.putString("Tune/Step 4 - Find Ideal Distance",
            "Drive manually to 1.5m from hub. Hold RT to shoot. Feed 5 balls. Count scores. "
            + "Repeat at 2.0m, 2.5m, 3.0m, 3.5m, 4.0m. "
            + "Set 'Ideal Distance (m)' to whichever distance scored the most balls. "
            + "This is where the robot will auto-park when you hold Y.");

        SmartDashboard.putString("Tune/Step 5 - Tune Shooter Speed",
            "CLOSED-LOOP: the shooter now holds exact RPM regardless of battery voltage. "
            + "Formula: target fraction = A + B * sqrt(distance), then velocity = fraction * 'Max RPS'. "
            + "Stand at your ideal distance. Hold RT. Watch 'Shooter At Speed' -- it should be TRUE. "
            + "If balls fall SHORT: increase 'Shooter B' by 0.02 or increase 'Max RPS' by 5. "
            + "If balls fly OVER: decrease 'Shooter B' by 0.02 or decrease 'Max RPS' by 5. "
            + "Test at 1m closer and 1m farther to verify. "
            + "'Shooter Target RPS' and 'Shooter Actual RPS' show live velocity. "
            + "They should match closely -- if Actual lags behind Target, the PID gains (kS/kV/kP in code) need tuning.");

        SmartDashboard.putString("Tune/Step 6 - Match Ready",
            "Once tuned, tell your programmer the final values for: "
            + "Ideal Distance, Shooter A, Shooter B, Shooter Max RPS, Aim Kp, Range Kp. "
            + "They will update the code defaults so these survive a reboot. "
            + "ALL changes on this dashboard take effect IMMEDIATELY -- no redeploy needed. "
            + "CONTROLS: Y+RT = AUTO-SCORE (aim+drive+shoot+auto-feed) | Y = auto-aim only | "
            + "X = align to hub tag | RT = shoot (auto-speed) | RT+D-Right = full power | "
            + "LT = intake | RB = feeder | LB = feeder reverse | D-Up/Down = arm. "
            + "DASHBOARD: 'Hub Active' = can you score? | 'Shift Timer' = seconds until next change | "
            + "'Fuel Scored' = estimated toward RP thresholds (100/360). "
            + "Controller RUMBLES 3s before your hub goes active -- pre-position!");
    }

    private void configureDashboard() {
        // ── MATCH TAB ── Primary view for drive coach during competition
        ShuffleboardTab match = Shuffleboard.getTab("Match");

        match.addBoolean("HUB ACTIVE", this::isHubActive)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FF0000"))
            .withSize(3, 3).withPosition(0, 0);

        match.addNumber("SHIFT TIMER", this::getSecondsUntilNextShiftChange)
            .withSize(2, 2).withPosition(3, 0);

        match.addBoolean("Auto-Cycle", () -> autoCycleActive)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#00AAFF", "Color when false", "#333333"))
            .withSize(2, 1).withPosition(5, 0);

        match.addNumber("Fuel Scored", () -> (double) estimatedFuelScored)
            .withSize(2, 1).withPosition(7, 0);

        match.addBoolean("Auto-Feed", () ->
                isAimToHubRequested() && isShooterRequested()
                && isShooterAtSpeed() && isHubActive())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(2, 1).withPosition(5, 1);

        match.addNumber("Need for Energized", () -> (double) Math.max(0, 100 - estimatedFuelScored))
            .withSize(2, 1).withPosition(7, 1);

        match.addBoolean("AIMED", () ->
                hadVisionPose && Math.abs(computeAimErrorDeg()) < kAimToHubDeadbandDeg)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FF4444"))
            .withSize(2, 1).withPosition(3, 2);

        match.addBoolean("AT SPEED", this::isShooterAtSpeed)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FF4444"))
            .withSize(2, 1).withPosition(5, 2);

        match.addNumber("Need for Supercharged", () -> (double) Math.max(0, 360 - estimatedFuelScored))
            .withSize(2, 1).withPosition(7, 2);

        match.addBoolean("WARNING: Hub Inactive", () -> !isHubActive() && isShooterRequested())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#333333"))
            .withSize(3, 1).withPosition(0, 3);

        match.addBoolean("Vision OK", () -> hadVisionPose)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(1, 1).withPosition(3, 3);

        match.addBoolean("RP: 100", () -> estimatedFuelScored >= 100)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#FFD700", "Color when false", "#333333"))
            .withSize(1, 1).withPosition(4, 3);

        match.addBoolean("RP: 360", () -> estimatedFuelScored >= 360)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#FFD700", "Color when false", "#333333"))
            .withSize(1, 1).withPosition(5, 3);

        match.addNumber("Hub Dist (m)", () -> lastDistanceToHubMeters)
            .withSize(2, 1).withPosition(6, 3);

        match.addNumber("Aim Error", this::computeAimErrorDeg)
            .withSize(1, 1).withPosition(8, 3);

        match.addBoolean("SAFE FOR AUTO", this::isSafeForAutoFunctions)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#FF0000"))
            .withSize(2, 1).withPosition(0, 4);

        match.addBoolean("Alliance Set", this::isAllianceKnown)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(1, 1).withPosition(2, 4);

        match.addBoolean("Vision Fresh", this::isVisionFresh)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(1, 1).withPosition(3, 4);

        match.addBoolean("Pose Valid", this::isPoseOnField)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(1, 1).withPosition(4, 4);

        // ── ROBOT TAB ── Detailed robot state + field map
        ShuffleboardTab robot = Shuffleboard.getTab("Robot");

        robot.add("Field", field2d)
            .withWidget(BuiltInWidgets.kField)
            .withSize(5, 3).withPosition(0, 0);

        robot.addNumber("Aim Error (deg)", this::computeAimErrorDeg)
            .withSize(2, 1).withPosition(5, 0);

        robot.addNumber("Target RPS", () -> shooterCommandedRps)
            .withSize(2, 1).withPosition(7, 0);

        robot.addNumber("Auto Shooter %", () -> computeDistanceBasedShooterOutput() * 100.0)
            .withSize(2, 1).withPosition(5, 1);

        robot.addNumber("Actual RPS", () -> shooterMotor.getVelocity().getValueAsDouble())
            .withSize(2, 1).withPosition(7, 1);

        robot.addNumber("Hub Distance (m)", () -> lastDistanceToHubMeters)
            .withSize(2, 1).withPosition(5, 2);

        robot.addBoolean("Vision Valid", () -> hadVisionPose)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(2, 1).withPosition(7, 2);

        robot.addString("Alliance", () -> {
            Optional<Alliance> a = DriverStation.getAlliance();
            return a.isPresent() ? a.get().name() : "UNKNOWN";
        }).withSize(2, 1).withPosition(0, 3);

        robot.addBoolean("Hub Active", this::isHubActive)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(2, 1).withPosition(2, 3);

        robot.addNumber("Match Time", DriverStation::getMatchTime)
            .withSize(2, 1).withPosition(4, 3);

        // ── VISION TAB ── Limelight camera feed
        ShuffleboardTab vision = Shuffleboard.getTab("Vision");
        vision.add("limelight-back", limelightCamera)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(6, 4)
            .withPosition(0, 0);

        Shuffleboard.selectTab("Match");
    }

    private void configureShooter() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = kShooterKs;
        config.Slot0.kV = kShooterKv;
        config.Slot0.kP = kShooterKp;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        shooterMotor.getConfigurator().apply(config);
        DriverStation.reportWarning(
            String.format("[SHOOTER] Velocity PID configured: kS=%.3f kV=%.3f kP=%.3f maxRPS=%.0f",
                kShooterKs, kShooterKv, kShooterKp, kShooterMaxVelocityRps),
            false
        );
    }

    private void configureLimelightStream() {
        String streamUrl = SmartDashboard.getString("Limelight Stream URL", kLimelightStreamUrlDefault);
        limelightCamera = new HttpCamera(
            kLimelightTableName,
            streamUrl,
            HttpCameraKind.kMJPGStreamer
        );
        CameraServer.addCamera(limelightCamera);
        LimelightHelpers.setupPortForwardingUSB(0);
        DriverStation.reportWarning("[LIMELIGHT] Stream URL: " + streamUrl, false);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(this::getDefaultDriveRequest)
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idleRequest).ignoringDisable(true)
        );

        // Clear controller rumble when disabled.
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
            driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
            operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }).ignoringDisable(true));

        // Reset fuel counter at match start (autonomous init).
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> {
            estimatedFuelScored = 0;
            fuelAccumulator = 0.0;
            lastShiftUpdateSec = -1.0;
        }));

        // Controls (same on driver & operator; operator has priority):
        // - Press A:         AUTO-CYCLE: robot drives to depot, intakes, drives to hub,
        //                    aims, shoots, repeats. Press A again or B to cancel.
        //                    Touch sticks to instantly override and take manual control.
        // - Hold LT:         intake
        // - Hold LB:         feeder reverse
        // - Hold RB:         feeder (manual)
        // - Hold RT:         shooter (auto-speed from distance to hub)
        // - Hold Y + RT:     AUTO-SCORE: aim + drive to hub + shoot + auto-feed when ready
        // - Hold D-pad Right + RT: shooter boost (full power override)
        // - Hold Y:          3D targeting (auto-aim + auto-drive to ideal range; stick orbits hub)
        // - Hold X:          AprilTag align (hub tags only; ignores non-hub tags)
        // - D-pad Up/Down:   intake arm up/down
        // - Left stick press: reseed field-centric heading
        // - Press B:         cancel auto-cycle
        // Dashboard: Hub Active, Shift Timer, Fuel Scored, RP tracking, Auto-Cycle status
        final Trigger intakeTrigger = new Trigger(this::isIntakeRequested);
        final Trigger feederReverseTrigger = new Trigger(this::isFeederReverseRequested);
        final Trigger feederTrigger = new Trigger(this::isFeederRequested);
        final Trigger shooterTrigger = new Trigger(this::isShooterRequested);
        final Trigger intakeArmUpTrigger = new Trigger(this::isIntakeArmUpRequested);
        final Trigger intakeArmDownTrigger = new Trigger(this::isIntakeArmDownRequested);

        intakeTrigger.whileTrue(Commands.runEnd(
            () -> {
                if (intakeMotor.getOutputCurrent() < kIntakeStallCurrentAmps) {
                    intakeMotor.set(kIntakePercentOutput);
                } else {
                    intakeMotor.set(0.0);
                    DriverStation.reportWarning("[INTAKE] Stall detected, motor stopped", false);
                }
                logIntakeDiagnostics();
            },
            () -> intakeMotor.set(0.0)
        ));
        intakeTrigger.onTrue(Commands.runOnce(() -> {
            lastIntakeDiagTimeSec = -1.0;
            logControl("LT detected -> Intake ON");
        }));
        intakeTrigger.onFalse(Commands.runOnce(() -> logControl("LT released -> Intake OFF")));

        feederTrigger.whileTrue(Commands.runEnd(
            () -> feederMotor.set(kFeederPercentOutput),
            () -> feederMotor.set(0.0)
        ));
        feederTrigger.onTrue(Commands.runOnce(() -> logControl("Feeder ON (manual or auto-feed)")));
        feederTrigger.onFalse(Commands.runOnce(() -> logControl("Feeder OFF")));

        feederReverseTrigger.whileTrue(Commands.runEnd(
            () -> feederMotor.set(kFeederReversePercentOutput),
            () -> feederMotor.set(0.0)
        ));
        feederReverseTrigger.onTrue(Commands.runOnce(() -> logControl("LB detected -> Feeder REVERSE ON")));
        feederReverseTrigger.onFalse(Commands.runOnce(() -> logControl("LB released -> Feeder REVERSE OFF")));

        // Shooter with distance-based auto-speed and vision pose updates every loop.
        // RT chooses target; D-pad Right overrides to full-power boost.
        RobotModeTriggers.teleop().whileTrue(Commands.run(() -> {
            updateVisionPose();
            if (!autoCycleActive) {
                final double targetPercent;
                if (shooterTrigger.getAsBoolean()) {
                    targetPercent = isShooterBoostRequested()
                        ? kShooterBoostPercentOutput
                        : computeDistanceBasedShooterOutput();
                } else {
                    targetPercent = 0.0;
                }
                updateShooterOutput(targetPercent);
            }
            updateShiftAwareTelemetry();
        }));
        shooterTrigger.onTrue(Commands.runOnce(() -> logControl("RT detected -> Shooter ON")));
        shooterTrigger.onFalse(Commands.runOnce(() -> logControl("RT released -> Shooter OFF")));

        intakeArmUpTrigger.whileTrue(Commands.runEnd(
            () -> setIntakeArmSafe(kIntakeArmUpPercentOutput),
            () -> intakeArmMotor.set(0.0)
        ));
        intakeArmUpTrigger.onTrue(Commands.runOnce(() -> logControl("D-pad Up detected -> Intake Arm CCW")));
        intakeArmUpTrigger.onFalse(Commands.runOnce(() -> logControl("D-pad Up released -> Intake Arm OFF")));

        intakeArmDownTrigger.whileTrue(Commands.runEnd(
            () -> setIntakeArmSafe(kIntakeArmDownPercentOutput),
            () -> intakeArmMotor.set(0.0)
        ));
        intakeArmDownTrigger.onTrue(Commands.runOnce(() -> logControl("D-pad Down detected -> Intake Arm CW")));
        intakeArmDownTrigger.onFalse(Commands.runOnce(() -> logControl("D-pad Down released -> Intake Arm OFF")));

        // Reset field-centric heading on left stick press.
        new Trigger(this::isReseedRequested).onTrue(Commands.sequence(
            Commands.runOnce(() -> logControl("Left stick pressed -> Reseed field-centric heading")),
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        ));

        // Debug intake arm position at 500ms cadence in teleop.
        RobotModeTriggers.teleop().whileTrue(Commands.run(this::maybeLogIntakeArmPosition));

        // ── Auto-Cycle: fully autonomous collect→score loop ──
        // A button: toggle auto-cycle on/off
        // B button: emergency cancel
        // Stick override: touching the sticks cancels auto-cycle instantly
        final Command autoCycle = createAutoCycleCommand();
        driverController.a().toggleOnTrue(autoCycle);
        driverController.b().onTrue(Commands.runOnce(() -> {
            if (autoCycle.isScheduled()) autoCycle.cancel();
        }));
        new Trigger(() -> autoCycleActive && hasManualDriveInput())
            .onTrue(Commands.runOnce(() -> {
                if (autoCycle.isScheduled()) autoCycle.cancel();
            }));

        // Cancel auto-cycle if safety conditions fail mid-cycle
        new Trigger(() -> autoCycleActive && !isSafeForAutoFunctions())
            .onTrue(Commands.runOnce(() -> {
                if (autoCycle.isScheduled()) autoCycle.cancel();
                DriverStation.reportWarning(
                    "[SAFETY] Auto-cycle cancelled: alliance=" + isAllianceKnown()
                    + " visionFresh=" + isVisionFresh() + " poseOnField=" + isPoseOnField(), false);
            }));

        // ── EMERGENCY STOP: START + BACK kills all motors instantly ──
        new Trigger(() -> driverController.start().getAsBoolean()
                && driverController.back().getAsBoolean())
            .onTrue(Commands.runOnce(() -> {
                shooterMotor.setControl(shooterNeutralRequest);
                shooterCommandedRps = 0;
                feederMotor.set(0);
                intakeMotor.set(0);
                intakeArmMotor.set(0);
                if (autoCycle.isScheduled()) autoCycle.cancel();
                DriverStation.reportWarning("[SAFETY] EMERGENCY STOP: all motors killed", false);
            }));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        if (!drivetrain.isAutoBuilderConfigured()) {
            DriverStation.reportError(
                "[AUTO] PathPlanner not configured. Save Robot Config in PathPlanner GUI (deploy/pathplanner/settings.json). Running fallback auto.",
                false
            );
            return createFallbackAutoCommand();
        }

        final Command pathCommand;
        try {
            pathCommand = new PathPlannerAuto(kCenterAutoName);
        } catch (Exception ex) {
            DriverStation.reportError(
                "[AUTO] Failed to load PathPlanner auto '" + kCenterAutoName + "'. Running fallback auto.",
                ex.getStackTrace()
            );
            return createFallbackAutoCommand();
        }

        return Commands.sequence(
            pathCommand,
            // Spin shooter up first.
            Commands.run(() -> updateShooterOutput(kShooterPercentOutput)).withTimeout(kAutoShooterSpinupSec),
            // Keep shooter spinning while feeding.
            Commands.deadline(
                Commands.waitSeconds(kAutoFeedSec),
                Commands.run(() -> updateShooterOutput(kShooterPercentOutput)),
                Commands.run(() -> feederMotor.set(kFeederPercentOutput))
            ),
            Commands.runOnce(() -> {
                feederMotor.set(0.0);
                updateShooterOutput(0.0);
            })
        );
    }

    private Command createFallbackAutoCommand() {
        return Commands.sequence(
            Commands.run(() -> updateShooterOutput(kShooterPercentOutput)).withTimeout(kAutoShooterSpinupSec),
            Commands.deadline(
                Commands.waitSeconds(kAutoFeedSec),
                Commands.run(() -> updateShooterOutput(kShooterPercentOutput)),
                Commands.run(() -> feederMotor.set(kFeederPercentOutput))
            ),
            Commands.runOnce(() -> {
                feederMotor.set(0.0);
                updateShooterOutput(0.0);
            })
        );
    }

    private void logControl(String message) {
        DriverStation.reportWarning("[CONTROLS] " + message, false);
    }

    private void logIntakeDiagnostics() {
        final double nowSec = Timer.getFPGATimestamp();
        if (lastIntakeDiagTimeSec > 0.0 && (nowSec - lastIntakeDiagTimeSec) < kIntakeDiagPeriodSec) {
            return;
        }
        lastIntakeDiagTimeSec = nowSec;

        DriverStation.reportWarning(String.format(
            "[INTAKE] cmd=%.2f applied=%.2f current=%.2fA bus=%.2fV temp=%.1fC err=%s",
            kIntakePercentOutput,
            intakeMotor.getAppliedOutput(),
            intakeMotor.getOutputCurrent(),
            intakeMotor.getBusVoltage(),
            intakeMotor.getMotorTemperature(),
            intakeMotor.getLastError()
        ), false);
    }

    private void updateShooterOutput(double targetFraction) {
        final double maxRps = SmartDashboard.getNumber("Tune/Shooter Max RPS", kShooterMaxVelocityRps);
        if (targetFraction <= 0.0) {
            shooterCommandedRps = 0.0;
            shooterMotor.setControl(shooterNeutralRequest);
            return;
        }
        shooterCommandedRps = targetFraction * maxRps;
        shooterMotor.setControl(shooterVelocityRequest.withVelocity(shooterCommandedRps));
    }

    public boolean isShooterAtSpeed() {
        if (shooterCommandedRps <= 0.0) return false;
        final double actualRps = shooterMotor.getVelocity().getValueAsDouble();
        return Math.abs(actualRps - shooterCommandedRps) < kShooterAtSpeedThresholdRps;
    }

    private double getRequestedRotationalRate() {
        // Priority 1: Hold X to auto-align to a visible AprilTag using Limelight tx.
        final boolean alignModeActive = isAlignRequested();
        if (alignModeActive != limelightAlignModeLast) {
            DriverStation.reportWarning(
                alignModeActive ? "[LIMELIGHT] X held -> align mode ON" : "[LIMELIGHT] X released -> align mode OFF",
                false
            );
            limelightAlignModeLast = alignModeActive;
        }

        if (alignModeActive) {
            final double tv = limelightTable.getEntry("tv").getDouble(0.0);
            if (tv >= 1.0 && isTrackedTagHubTag()) {
                final double txDeg = limelightTable.getEntry("tx").getDouble(0.0);
                if (!limelightHadTagLast) {
                    DriverStation.reportWarning("[LIMELIGHT] Hub tag detected", false);
                    limelightHadTagLast = true;
                }

                if (Math.abs(txDeg) < kLimelightTxDeadbandDeg) {
                    maybeLogLimelightDebug(txDeg, 0.0);
                    return 0.0;
                }
                final double txRad = Math.toRadians(txDeg);
                final double rawTurnRate = -kLimelightTurnKp * txRad;
                final double turnRate = Math.max(
                    -kLimelightMaxTurnRateRadPerSec,
                    Math.min(kLimelightMaxTurnRateRadPerSec, rawTurnRate)
                );
                maybeLogLimelightDebug(txDeg, turnRate);
                return turnRate;
            }
            if (limelightHadTagLast) {
                DriverStation.reportWarning("[LIMELIGHT] No hub tag in view", false);
                limelightHadTagLast = false;
            }
            return 0.0;
        }

        limelightHadTagLast = false;

        // Priority 2: Hold Y to auto-aim toward hub using fused field pose.
        final boolean aimToHubActive = isAimToHubRequested();
        if (aimToHubActive != aimToHubModeLast) {
            DriverStation.reportWarning(
                aimToHubActive ? "[AIM] auto-aim to hub ON" : "[AIM] auto-aim to hub OFF",
                false
            );
            aimToHubModeLast = aimToHubActive;
        }

        if (aimToHubActive) {
            return computeAimToHubTurnRate();
        }

        // Priority 3: Manual right-stick rotation.
        return -getRequestedRightX() * MaxAngularRate;
    }

    private SwerveRequest getDefaultDriveRequest() {
        final double rawTurnRateRadPerSec = getRequestedRotationalRate();

        // 3D targeting: when Y (aim-to-hub) is held AND we have valid vision,
        // auto-drive to the ideal shooting distance. Driver stick perpendicular
        // to the hub direction is preserved so the driver can orbit the hub.
        if (isAimToHubRequested() && isSafeForAutoFunctions() && lastDistanceToHubMeters > 0) {
            final double[] autoVel = computeAutoPositionVelocity();
            final double xVelMps = xVelLimiter.calculate(autoVel[0]);
            final double yVelMps = yVelLimiter.calculate(autoVel[1]);
            final double turnRate = rotVelLimiter.calculate(rawTurnRateRadPerSec);

            final boolean isIdle = Math.abs(xVelMps) < kIdleLinearSpeedThresholdMps
                && Math.abs(yVelMps) < kIdleLinearSpeedThresholdMps
                && Math.abs(turnRate) < kIdleTurnRateThresholdRadPerSec;

            if (isIdle) {
                xVelLimiter.reset(0.0);
                yVelLimiter.reset(0.0);
                rotVelLimiter.reset(0.0);
                return brakeRequest;
            }

            return drive.withVelocityX(xVelMps)
                .withVelocityY(yVelMps)
                .withRotationalRate(turnRate);
        }

        final double rawXVelMps = -getRequestedLeftY() * MaxSpeed;
        final double rawYVelMps = -getRequestedLeftX() * MaxSpeed;

        final boolean isIdleCommand = Math.abs(rawXVelMps) < kIdleLinearSpeedThresholdMps
            && Math.abs(rawYVelMps) < kIdleLinearSpeedThresholdMps
            && Math.abs(rawTurnRateRadPerSec) < kIdleTurnRateThresholdRadPerSec;

        if (isIdleCommand) {
            xVelLimiter.reset(0.0);
            yVelLimiter.reset(0.0);
            rotVelLimiter.reset(0.0);
            return brakeRequest;
        }

        final double xVelMps = xVelLimiter.calculate(rawXVelMps);
        final double yVelMps = yVelLimiter.calculate(rawYVelMps);
        final double turnRateRadPerSec = rotVelLimiter.calculate(rawTurnRateRadPerSec);

        return drive.withVelocityX(xVelMps)
            .withVelocityY(yVelMps)
            .withRotationalRate(turnRateRadPerSec);
    }

    /**
     * Computes field-centric X/Y velocities that drive the robot to the ideal
     * shooting distance from the hub.  The radial component (toward/away from
     * hub) is fully automatic.  The tangential component (orbit around the
     * hub) is left to the driver's left-stick so they can adjust their angle.
     */
    private double[] computeAutoPositionVelocity() {
        final Pose2d pose = drivetrain.getState().Pose;
        final Translation2d hubPos = getAllianceHubPosition();
        final double dx = hubPos.getX() - pose.getX();
        final double dy = hubPos.getY() - pose.getY();
        final double dist = Math.hypot(dx, dy);
        if (dist < 0.01) {
            return new double[]{0.0, 0.0};
        }

        // Unit vector from robot toward the hub (radial direction).
        final double ux = dx / dist;
        final double uy = dy / dist;

        // --- Radial: close the range error ---
        final double idealDist = SmartDashboard.getNumber("Tune/Ideal Distance (m)", kIdealShootingDistanceMeters);
        final double rangeKp = SmartDashboard.getNumber("Tune/Range Kp", kRangeKp);

        final double rangeError = dist - idealDist;
        double radialSpeed = 0.0;
        if (Math.abs(rangeError) > kRangeDistDeadbandMeters) {
            radialSpeed = MathUtil.clamp(rangeError * rangeKp,
                -kMaxRangeSpeedMps, kMaxRangeSpeedMps);
        }

        // --- Tangential: driver stick projected perpendicular to hub line ---
        // Perpendicular unit vector (90-deg CCW): (-uy, ux)
        final double driverX = -getRequestedLeftY() * MaxSpeed;
        final double driverY = -getRequestedLeftX() * MaxSpeed;
        final double tangentialInput = driverX * (-uy) + driverY * ux;
        final double tangentialSpeed = MathUtil.clamp(tangentialInput,
            -kMaxStrafeSpeedMps, kMaxStrafeSpeedMps);

        // Compose field-centric velocity: radial + tangential.
        final double vx = radialSpeed * ux + tangentialSpeed * (-uy);
        final double vy = radialSpeed * uy + tangentialSpeed * ux;

        return new double[]{vx, vy};
    }

    private double getRequestedLeftY() {
        final double operatorAxis = isOperatorConnected() ? operatorController.getLeftY() : 0.0;
        return getPrioritizedAxis(driverController.getLeftY(), operatorAxis);
    }

    private double getRequestedLeftX() {
        final double operatorAxis = isOperatorConnected() ? operatorController.getLeftX() : 0.0;
        return getPrioritizedAxis(driverController.getLeftX(), operatorAxis);
    }

    private double getRequestedRightX() {
        final double operatorAxis = isOperatorConnected() ? operatorController.getRightX() : 0.0;
        return getPrioritizedAxis(driverController.getRightX(), operatorAxis);
    }

    private double getPrioritizedAxis(double driverAxis, double operatorAxis) {
        driverAxis = MathUtil.applyDeadband(driverAxis, kInputDeadband);
        operatorAxis = MathUtil.applyDeadband(operatorAxis, kInputDeadband);
        if (isOperatorControllingRobot()) {
            return operatorAxis * kOperatorDriveSpeedScale;
        }
        return driverAxis;
    }

    private boolean isOperatorControllingRobot() {
        if (!isOperatorConnected()) {
            operatorOverrideLatched = false;
            return false;
        }
        final double leftX = Math.abs(operatorController.getLeftX());
        final double leftY = Math.abs(operatorController.getLeftY());
        final double rightX = Math.abs(operatorController.getRightX());
        final double leftTrigger = operatorController.getLeftTriggerAxis();
        final double rightTrigger = operatorController.getRightTriggerAxis();
        final boolean buttonIntent = operatorController.leftBumper().getAsBoolean()
            || operatorController.rightBumper().getAsBoolean()
            || operatorController.povUp().getAsBoolean()
            || operatorController.povDown().getAsBoolean()
            || operatorController.povRight().getAsBoolean()
            || operatorController.leftStick().getAsBoolean()
            || operatorController.x().getAsBoolean()
            || operatorController.y().getAsBoolean();

        final boolean highIntent = leftX > kOperatorOverrideOnThreshold
            || leftY > kOperatorOverrideOnThreshold
            || rightX > kOperatorOverrideOnThreshold
            || leftTrigger > kOperatorOverrideOnThreshold
            || rightTrigger > kOperatorOverrideOnThreshold
            || buttonIntent;
        final boolean lowIntent = leftX > kOperatorOverrideOffThreshold
            || leftY > kOperatorOverrideOffThreshold
            || rightX > kOperatorOverrideOffThreshold
            || leftTrigger > kOperatorOverrideOffThreshold
            || rightTrigger > kOperatorOverrideOffThreshold
            || buttonIntent;

        // Latch with hysteresis so tiny controller noise does not rapidly steal/release control.
        operatorOverrideLatched = operatorOverrideLatched ? lowIntent : highIntent;
        return operatorOverrideLatched;
    }

    private boolean isInputFromOperator(boolean driverInput, boolean operatorInput) {
        return isOperatorControllingRobot() ? operatorInput : driverInput;
    }

    private boolean isIntakeRequested() {
        return isInputFromOperator(
            driverController.getLeftTriggerAxis() > 0.2,
            isOperatorConnected() && operatorController.getLeftTriggerAxis() > 0.2
        );
    }

    private boolean isFeederRequested() {
        boolean manual = isInputFromOperator(
            driverController.rightBumper().getAsBoolean(),
            isOperatorConnected() && operatorController.rightBumper().getAsBoolean()
        );
        boolean autoFeed = isAimToHubRequested()
            && isShooterRequested()
            && isShooterAtSpeed()
            && isHubActive()
            && isSafeForAutoFunctions()
            && isVisionAimConfirmed();
        return manual || autoFeed;
    }

    private boolean isFeederReverseRequested() {
        return isInputFromOperator(
            driverController.leftBumper().getAsBoolean(),
            isOperatorConnected() && operatorController.leftBumper().getAsBoolean()
        );
    }

    private boolean isShooterRequested() {
        return isInputFromOperator(
            driverController.getRightTriggerAxis() > 0.2,
            isOperatorConnected() && operatorController.getRightTriggerAxis() > 0.2
        );
    }

    private boolean isShooterBoostRequested() {
        return isInputFromOperator(
            driverController.povRight().getAsBoolean(),
            isOperatorConnected() && operatorController.povRight().getAsBoolean()
        );
    }

    private boolean isIntakeArmUpRequested() {
        return isInputFromOperator(
            driverController.povUp().getAsBoolean(),
            isOperatorConnected() && operatorController.povUp().getAsBoolean()
        );
    }

    private boolean isIntakeArmDownRequested() {
        return isInputFromOperator(
            driverController.povDown().getAsBoolean(),
            isOperatorConnected() && operatorController.povDown().getAsBoolean()
        );
    }

    private boolean isAlignRequested() {
        return isInputFromOperator(
            driverController.x().getAsBoolean(),
            isOperatorConnected() && operatorController.x().getAsBoolean()
        );
    }

    private boolean isAimToHubRequested() {
        return isInputFromOperator(
            driverController.y().getAsBoolean(),
            isOperatorConnected() && operatorController.y().getAsBoolean()
        );
    }

    private boolean isReseedRequested() {
        return isInputFromOperator(
            driverController.leftStick().getAsBoolean(),
            isOperatorConnected() && operatorController.leftStick().getAsBoolean()
        );
    }

    private boolean isOperatorConnected() {
        return DriverStation.isJoystickConnected(1);
    }

    private boolean isAllianceKnown() {
        return DriverStation.getAlliance().isPresent();
    }

    private boolean isVisionFresh() {
        if (!hadVisionPose) return false;
        return (Timer.getFPGATimestamp() - lastVisionTagTimeSec) < kVisionFreshnessTimeoutSec;
    }

    private boolean isPoseOnField() {
        final Pose2d pose = drivetrain.getState().Pose;
        return pose.getX() > -kFieldMarginMeters
            && pose.getX() < kFieldLengthMeters + kFieldMarginMeters
            && pose.getY() > -kFieldMarginMeters
            && pose.getY() < kFieldWidthMeters + kFieldMarginMeters;
    }

    private boolean isSafeForAutoFunctions() {
        return isAllianceKnown() && isVisionFresh() && isPoseOnField();
    }

    private void updateVisionPose() {
        // MegaTag2 pose estimation is handled by CommandSwerveDrivetrain.periodic().
        // Here we just read the fused pose from the drivetrain to compute hub distance.
        updateHubTagFilter();

        // Only trust the pose for aiming/distance if the Limelight has actually seen tags.
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightTableName);
        if (mt2 != null && mt2.tagCount > 0) {
            hadVisionPose = true;
            lastVisionTagTimeSec = Timer.getFPGATimestamp();
        }

        final Pose2d fusedPose = drivetrain.getState().Pose;
        final Translation2d hubPos = getAllianceHubPosition();
        lastDistanceToHubMeters = fusedPose.getTranslation().getDistance(hubPos);

        field2d.setRobotPose(fusedPose);
        field2d.getObject("Hub").setPose(new Pose2d(hubPos, new Rotation2d()));

        maybeLogVisionPose(fusedPose);
    }

    private boolean hubTagFilterSetForRed = false;
    private boolean hubTagFilterApplied = false;

    private void updateHubTagFilter() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return;
        }
        final boolean isRed = alliance.get() == Alliance.Red;
        if (hubTagFilterApplied && hubTagFilterSetForRed == isRed) {
            return;
        }
        hubTagFilterSetForRed = isRed;
        hubTagFilterApplied = true;
        // Don't filter for MegaTag2 pose estimation (it uses all tags).
        // But set the priority tag to the centered front-face tag on our hub
        // so raw tx/ty align to the hub opening.
        int priorityTag = isRed ? kRedHubTagIds[0] : kBlueHubTagIds[0];
        LimelightHelpers.setPriorityTagID(kLimelightTableName, priorityTag);
        DriverStation.reportWarning(
            "[LIMELIGHT] Alliance=" + (isRed ? "RED" : "BLUE") + " priorityTag=" + priorityTag,
            false
        );
    }

    private double computeAimErrorDeg() {
        final Pose2d pose = drivetrain.getState().Pose;
        final Translation2d hubPos = getAllianceHubPosition();
        final double desiredRad = Math.atan2(
            hubPos.getY() - pose.getY(),
            hubPos.getX() - pose.getX()
        ) + Math.toRadians(kShooterFacingOffsetDeg);
        final double errorRad = new Rotation2d(desiredRad).minus(pose.getRotation()).getRadians();
        return Math.toDegrees(errorRad);
    }

    private double computeDistanceBasedShooterOutput() {
        if (!hadVisionPose) {
            return kShooterPercentOutput;
        }
        final double a = SmartDashboard.getNumber("Tune/Shooter A", kShooterDistA);
        final double b = SmartDashboard.getNumber("Tune/Shooter B", kShooterDistB);
        final double min = SmartDashboard.getNumber("Tune/Shooter Min %", kShooterDistMinOutput);
        final double max = SmartDashboard.getNumber("Tune/Shooter Max %", kShooterDistMaxOutput);
        final double raw = a + b * Math.sqrt(Math.max(0, lastDistanceToHubMeters));
        return Math.max(min, Math.min(max, raw));
    }

    private double computeAimToHubTurnRate() {
        if (!isVisionFresh()) {
            DriverStation.reportWarning("[AIM] Vision stale or never seen, aim-to-hub disabled", false);
            return 0.0;
        }

        final double aimKp = SmartDashboard.getNumber("Tune/Aim Kp", kAimToHubKp);

        final Pose2d currentPose = drivetrain.getState().Pose;
        final Translation2d hubPos = getAllianceHubPosition();
        final double dx = hubPos.getX() - currentPose.getX();
        final double dy = hubPos.getY() - currentPose.getY();
        final double desiredHeadingRad = Math.atan2(dy, dx) + Math.toRadians(kShooterFacingOffsetDeg);

        final Rotation2d desiredRotation = new Rotation2d(desiredHeadingRad);
        final double errorRad = desiredRotation.minus(currentPose.getRotation()).getRadians();

        if (Math.abs(Math.toDegrees(errorRad)) < kAimToHubDeadbandDeg) {
            return 0.0;
        }

        final double rawTurnRate = aimKp * errorRad;
        return Math.max(-kAimToHubMaxTurnRateRadPerSec, Math.min(kAimToHubMaxTurnRateRadPerSec, rawTurnRate));
    }

    private Translation2d getAllianceHubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return kRedHubPosition;
        }
        return kBlueHubPosition;
    }

    /**
     * Returns true if the Limelight's currently tracked fiducial (tid) is
     * one of the hub AprilTag IDs for either alliance.  This prevents the
     * X-button align from locking onto trench, outpost, or other non-hub tags.
     */
    private boolean isTrackedTagHubTag() {
        final int tid = (int) LimelightHelpers.getFiducialID(kLimelightTableName);
        if (tid < 0) {
            return false;
        }
        for (int id : kBlueHubTagIds) {
            if (id == tid) return true;
        }
        for (int id : kRedHubTagIds) {
            if (id == tid) return true;
        }
        return false;
    }

    private boolean isHubActive() {
        if (DriverStation.isAutonomous()) {
            return true;
        }

        final double matchTime = DriverStation.getMatchTime();

        if (matchTime <= 30.0) {
            return true;
        }

        if (matchTime > 130.0) {
            return true;
        }

        final String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) {
            return true;
        }

        final boolean weAreRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        final boolean redGoesFirstInactive = gameData.charAt(0) == 'R';
        final boolean weGoFirstInactive = (weAreRed == redGoesFirstInactive);

        // Shifts 1-4: 25s each, from 130s down to 30s
        final int shiftIndex = (int) ((130.0 - matchTime) / 25.0);
        final boolean oddShift = (shiftIndex % 2 == 0);

        if (weGoFirstInactive) {
            return !oddShift;
        } else {
            return oddShift;
        }
    }

    // ── Smart Cycle Detection ──

    private boolean isHopperLikelyEmpty() {
        if (Math.abs(feederMotor.getOutputCurrent()) > kFeederEmptyCurrentAmps) {
            feederLowCurrentStartSec = -1.0;
            return false;
        }
        if (feederLowCurrentStartSec < 0) {
            feederLowCurrentStartSec = Timer.getFPGATimestamp();
        }
        return (Timer.getFPGATimestamp() - feederLowCurrentStartSec) >= kFeederEmptyConfirmSec;
    }

    private boolean isCollectionDone() {
        if (Math.abs(intakeMotor.getOutputCurrent()) > kIntakeIdleCurrentAmps) {
            intakeIdleStartSec = -1.0;
            return false;
        }
        if (intakeIdleStartSec < 0) {
            intakeIdleStartSec = Timer.getFPGATimestamp();
        }
        return (Timer.getFPGATimestamp() - intakeIdleStartSec) >= kIntakeIdleConfirmSec;
    }

    private boolean isVisionAimConfirmed() {
        final double tv = limelightTable.getEntry("tv").getDouble(0.0);
        if (tv >= 1.0 && isTrackedTagHubTag()) {
            return Math.abs(limelightTable.getEntry("tx").getDouble(99.0)) < kAutoFeedAimThresholdDeg;
        }
        return Math.abs(computeAimErrorDeg()) < kAutoFeedAimThresholdDeg;
    }

    // ── Auto-Cycle: button-driven autonomous teleop ──

    private Translation2d getAllianceDepotPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return kRedDepotPosition;
        }
        return kBlueDepotPosition;
    }

    private Pose2d computeDepotPose() {
        final Translation2d depot = getAllianceDepotPosition();
        final Translation2d hub = getAllianceHubPosition();
        final double heading = Math.atan2(
            hub.getY() - depot.getY(), hub.getX() - depot.getX());
        return new Pose2d(depot, new Rotation2d(heading));
    }

    private Pose2d computeScoringPose() {
        final Translation2d hub = getAllianceHubPosition();
        final Translation2d depot = getAllianceDepotPosition();
        final double dx = hub.getX() - depot.getX();
        final double dy = hub.getY() - depot.getY();
        final double dist = Math.hypot(dx, dy);
        final double ux = dx / dist;
        final double uy = dy / dist;
        final double idealDist = SmartDashboard.getNumber(
            "Tune/Ideal Distance (m)", kIdealShootingDistanceMeters);
        return new Pose2d(
            hub.getX() - ux * idealDist,
            hub.getY() - uy * idealDist,
            new Rotation2d(Math.atan2(uy, ux)));
    }

    private SwerveRequest computeAutoScoreDriveRequest() {
        if (!isSafeForAutoFunctions() || lastDistanceToHubMeters <= 0) {
            return brakeRequest;
        }

        // Vision-refined aiming: use Limelight tx directly when a hub tag is visible
        // for sub-degree precision. Fall back to pose-based aim otherwise.
        double rawTurnRate;
        final double tv = limelightTable.getEntry("tv").getDouble(0.0);
        if (tv >= 1.0 && isTrackedTagHubTag()) {
            final double txRad = Math.toRadians(limelightTable.getEntry("tx").getDouble(0.0));
            rawTurnRate = MathUtil.clamp(-kLimelightTurnKp * txRad,
                -kLimelightMaxTurnRateRadPerSec, kLimelightMaxTurnRateRadPerSec);
        } else {
            rawTurnRate = computeAimToHubTurnRate();
        }

        final double turnRate = rotVelLimiter.calculate(rawTurnRate);
        final double[] autoVel = computeAutoPositionVelocity();
        final double xVel = xVelLimiter.calculate(autoVel[0]);
        final double yVel = yVelLimiter.calculate(autoVel[1]);

        if (Math.abs(xVel) < kIdleLinearSpeedThresholdMps
            && Math.abs(yVel) < kIdleLinearSpeedThresholdMps
            && Math.abs(turnRate) < kIdleTurnRateThresholdRadPerSec) {
            xVelLimiter.reset(0);
            yVelLimiter.reset(0);
            rotVelLimiter.reset(0);
            return brakeRequest;
        }
        return drive.withVelocityX(xVel).withVelocityY(yVel)
            .withRotationalRate(turnRate);
    }

    private boolean hasManualDriveInput() {
        boolean driver = Math.abs(driverController.getLeftX()) > kManualOverrideThreshold
            || Math.abs(driverController.getLeftY()) > kManualOverrideThreshold
            || Math.abs(driverController.getRightX()) > kManualOverrideThreshold;
        boolean operator = isOperatorConnected()
            && (Math.abs(operatorController.getLeftX()) > kManualOverrideThreshold
                || Math.abs(operatorController.getLeftY()) > kManualOverrideThreshold
                || Math.abs(operatorController.getRightX()) > kManualOverrideThreshold);
        return driver || operator;
    }

    private Command createAutoCycleCommand() {
        return Commands.either(
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> {
                        autoCycleActive = true;
                        DriverStation.reportWarning("[AUTO-CYCLE] Started", false);
                    }),
                    Commands.sequence(
                        // ── COLLECT PHASE ──
                        // Drive to depot with intake running, collecting balls on the way
                        Commands.runOnce(() -> {
                            intakeIdleStartSec = -1.0;
                            collectPhaseStartSec = Timer.getFPGATimestamp();
                            DriverStation.reportWarning("[AUTO-CYCLE] Collecting fuel", false);
                        }),
                        Commands.defer(() -> Commands.deadline(
                            AutoBuilder.pathfindToPose(computeDepotPose(), pathfindConstraints),
                            Commands.run(() -> intakeMotor.set(kIntakePercentOutput))
                        ), Set.of(drivetrain)),
                        // Smart collect: stop when intake current drops (no more balls)
                        // or max timeout as fallback
                        Commands.run(() -> intakeMotor.set(kIntakePercentOutput))
                            .until(() -> {
                                double elapsed = Timer.getFPGATimestamp() - collectPhaseStartSec;
                                return elapsed >= kMinCollectSec && isCollectionDone();
                            })
                            .withTimeout(kAutoIntakeMaxSec),
                        Commands.runOnce(() -> intakeMotor.set(0)),

                        // ── SCORE PHASE ──
                        // Drive to hub while pre-spinning the shooter (saves ~2s per cycle)
                        Commands.runOnce(() -> {
                            feederLowCurrentStartSec = -1.0;
                            scoringPhaseStartSec = Timer.getFPGATimestamp();
                            DriverStation.reportWarning("[AUTO-CYCLE] Driving to score", false);
                        }),
                        Commands.defer(() -> Commands.deadline(
                            AutoBuilder.pathfindToPose(computeScoringPose(), pathfindConstraints),
                            Commands.run(() -> {
                                updateShooterOutput(computeDistanceBasedShooterOutput());
                                updateVisionPose();
                            })
                        ), Set.of(drivetrain)),
                        // Vision-refined scoring: aim with Limelight tx, feed only when
                        // camera CONFIRMS aim, stop when hopper is empty (current-based)
                        Commands.runOnce(() ->
                            DriverStation.reportWarning("[AUTO-CYCLE] Scoring", false)),
                        drivetrain.run(() -> {
                            drivetrain.setControl(computeAutoScoreDriveRequest());
                            updateShooterOutput(computeDistanceBasedShooterOutput());
                            updateVisionPose();
                            boolean ready = isShooterAtSpeed() && isHubActive()
                                && isSafeForAutoFunctions()
                                && isVisionAimConfirmed();
                            feederMotor.set(ready ? kFeederPercentOutput : 0);
                        }).until(() -> {
                            double elapsed = Timer.getFPGATimestamp() - scoringPhaseStartSec;
                            return elapsed >= kMinScoringSec && isHopperLikelyEmpty();
                        }).withTimeout(kAutoScoreMaxSec),
                        Commands.runOnce(() -> {
                            feederMotor.set(0);
                            updateShooterOutput(0);
                        })
                    ).repeatedly()
                ).finallyDo(() -> {
                    autoCycleActive = false;
                    feederMotor.set(0);
                    intakeMotor.set(0);
                    updateShooterOutput(0);
                    DriverStation.reportWarning("[AUTO-CYCLE] Stopped", false);
                }).withTimeout(kAutoCycleTimeoutSec),
                Commands.runOnce(() -> DriverStation.reportError(
                    "[AUTO-CYCLE] BLOCKED: alliance unknown, vision stale, or pose off-field", false)),
                this::isSafeForAutoFunctions
            ),
            Commands.runOnce(() -> DriverStation.reportError(
                "[AUTO-CYCLE] Cannot start: PathPlanner not configured", false)),
            drivetrain::isAutoBuilderConfigured
        );
    }

    private double getSecondsUntilNextShiftChange() {
        if (DriverStation.isAutonomous()) return -1.0;
        final double matchTime = DriverStation.getMatchTime();
        if (matchTime <= 0) return -1.0;
        if (matchTime <= 30.0) return -1.0;
        if (matchTime > 130.0) return matchTime - 130.0;
        double timeIntoShifts = 130.0 - matchTime;
        double progressInCurrentShift = timeIntoShifts % 25.0;
        return 25.0 - progressInCurrentShift;
    }

    private void updateShiftAwareTelemetry() {
        final double nowSec = Timer.getFPGATimestamp();
        final double dt = (lastShiftUpdateSec > 0) ? (nowSec - lastShiftUpdateSec) : 0.0;
        lastShiftUpdateSec = nowSec;

        final boolean hubActive = isHubActive();
        final double secUntilChange = getSecondsUntilNextShiftChange();

        boolean shouldRumble = !hubActive
            && secUntilChange >= 0
            && secUntilChange <= kRumblePreActiveSec;
        driverController.getHID().setRumble(
            GenericHID.RumbleType.kBothRumble, shouldRumble ? kRumbleIntensity : 0.0);
        if (isOperatorConnected()) {
            operatorController.getHID().setRumble(
                GenericHID.RumbleType.kBothRumble, shouldRumble ? kRumbleIntensity : 0.0);
        }

        boolean scoring = isFeederRequested() && isShooterAtSpeed() && hubActive;
        if (scoring && dt > 0) {
            fuelAccumulator += kEstimatedFuelPerSec * dt;
            estimatedFuelScored = (int) fuelAccumulator;
        }
    }

    private void maybeLogVisionPose(Pose2d pose) {
        final double nowSec = Timer.getFPGATimestamp();
        if (lastVisionPoseLogTimeSec > 0.0 && (nowSec - lastVisionPoseLogTimeSec) < kVisionPoseLogPeriodSec) {
            return;
        }
        lastVisionPoseLogTimeSec = nowSec;

        final double autoSpeedPct = computeDistanceBasedShooterOutput() * 100.0;
        DriverStation.reportWarning(
            String.format(
                "[VISION] x=%.2f y=%.2f yaw=%.1f° | dist=%.2fm | autoSpeed=%.0f%% | hubActive=%s",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees(),
                lastDistanceToHubMeters, autoSpeedPct,
                isHubActive() ? "YES" : "NO"
            ),
            false
        );
    }

    private void maybeLogIntakeArmPosition() {
        final double nowSec = Timer.getFPGATimestamp();
        if (lastIntakeArmPosLogTimeSec > 0.0 && (nowSec - lastIntakeArmPosLogTimeSec) < kIntakeArmPosLogPeriodSec) {
            return;
        }
        lastIntakeArmPosLogTimeSec = nowSec;

        final double rotations = intakeArmMotor.getEncoder().getPosition();
        final double degrees = rotations * 360.0;
        DriverStation.reportWarning(
            String.format("[INTAKE_ARM] pos=%.3f rotations (%.1f deg)", rotations, degrees),
            false
        );
    }

    private void setIntakeArmSafe(double output) {
        final double pos = intakeArmMotor.getEncoder().getPosition();
        final double current = intakeArmMotor.getOutputCurrent();

        if (current > kIntakeArmStallCurrentAmps) {
            intakeArmMotor.set(0.0);
            DriverStation.reportWarning("[INTAKE_ARM] Stall detected, motor stopped", false);
            return;
        }

        if (output < 0 && pos <= kIntakeArmMinRotations) {
            intakeArmMotor.set(0.0);
            DriverStation.reportWarning("[INTAKE_ARM] At min soft limit, stopping", false);
            return;
        }
        if (output > 0 && pos >= kIntakeArmMaxRotations) {
            intakeArmMotor.set(0.0);
            DriverStation.reportWarning("[INTAKE_ARM] At max soft limit, stopping", false);
            return;
        }

        intakeArmMotor.set(output);
    }

    private void maybeLogLimelightDebug(double txDeg, double turnRateRadPerSec) {
        final double nowSec = Timer.getFPGATimestamp();
        if (lastLimelightDebugTimeSec > 0.0 && (nowSec - lastLimelightDebugTimeSec) < 0.25) {
            return;
        }
        lastLimelightDebugTimeSec = nowSec;
        DriverStation.reportWarning(
            String.format("[LIMELIGHT] tx=%.2f deg, autoTurn=%.2f rad/s", txDeg, turnRateRadPerSec),
            false
        );
    }
}
