// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Optional;

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
    private static final double kShooterStartPercentOutput = 0.05;
    private static final double kShooterRampUpDurationSec = 0.5;
    private static final double kShooterRampDownDurationSec = 0.5;
    private static final double kShooterRampSettledFraction = 0.99;
    private static final double kIntakeArmUpPercentOutput = -0.3;   // counterclockwise
    private static final double kIntakeArmDownPercentOutput = 0.3;  // clockwise
    private static final double kIntakeArmMinRotations = -2.0;     // soft limit (tune on robot)
    private static final double kIntakeArmMaxRotations = 2.0;      // soft limit (tune on robot)
    private static final double kIntakeArmStallCurrentAmps = 35.0; // current above this = stalled
    private static final double kIntakeStallCurrentAmps = 40.0;    // intake roller stall protection
    private static final double kIntakeDiagPeriodSec = 1.0;
    private static final String kLimelightTableName = "limelight-back";
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

    // 2026 REBUILT field dimensions (blue-origin WPILib coordinates, meters)
    private static final double kFieldLengthMeters = 16.54;
    private static final double kFieldWidthMeters = 8.07;
    private static final Translation2d kBlueHubPosition = new Translation2d(4.03, kFieldWidthMeters / 2.0);
    private static final Translation2d kRedHubPosition = new Translation2d(kFieldLengthMeters - 4.03, kFieldWidthMeters / 2.0);

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
    private final DutyCycleOut shooterRequest = new DutyCycleOut(0);
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(kLimelightTableName);
    private double lastIntakeDiagTimeSec = -1.0;
    private double lastIntakeArmPosLogTimeSec = -1.0;
    private double shooterAppliedPercent = 0.0;
    private double lastShooterUpdateTimeSec = Timer.getFPGATimestamp();
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
    private boolean aimToHubModeLast = false;

    private final Field2d field2d = new Field2d();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
        configureLimelightStream();
        SmartDashboard.putData("Field", field2d);
        DriverStation.reportWarning(
            "[AUTO] PathPlanner configured: " + drivetrain.isAutoBuilderConfigured(),
            false
        );
    }

    private void configureLimelightStream() {
        HttpCamera limelightFeed = new HttpCamera(
            kLimelightTableName,
            "http://" + kLimelightTableName + ".local:5800/stream.mjpeg",
            HttpCameraKind.kMJPGStreamer
        );
        CameraServer.addCamera(limelightFeed);
        // Forward Limelight ports so the stream is reachable through the roboRIO over FMS.
        LimelightHelpers.setupPortForwardingUSB(0);
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

        // Controls (same on driver & operator; operator has priority):
        // - Hold LT:         intake
        // - Hold LB:         feeder reverse
        // - Hold RB:         feeder
        // - Hold RT:         shooter (auto-speed from distance to hub)
        // - Hold D-pad Right + RT: shooter boost (full power override)
        // - Hold Y:          3D targeting (auto-aim + auto-drive to ideal range; stick orbits hub)
        // - Hold X:          AprilTag align (hub tags only; ignores non-hub tags)
        // - D-pad Up/Down:   intake arm up/down
        // - Left stick press: reseed field-centric heading
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
        feederTrigger.onTrue(Commands.runOnce(() -> logControl("RB detected -> Feeder ON")));
        feederTrigger.onFalse(Commands.runOnce(() -> logControl("RB released -> Feeder OFF")));

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
            final double targetPercent;
            if (shooterTrigger.getAsBoolean()) {
                targetPercent = isShooterBoostRequested()
                    ? kShooterBoostPercentOutput
                    : computeDistanceBasedShooterOutput();
            } else {
                targetPercent = 0.0;
            }
            updateShooterOutput(targetPercent);
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

    private void updateShooterOutput(double targetPercent) {
        final double nowSec = Timer.getFPGATimestamp();
        final double dtSec = Math.max(0.0, nowSec - lastShooterUpdateTimeSec);
        lastShooterUpdateTimeSec = nowSec;

        final double rampDurationSec = targetPercent > shooterAppliedPercent
            ? kShooterRampUpDurationSec
            : kShooterRampDownDurationSec;
        final double tauSec = -rampDurationSec / Math.log(1.0 - kShooterRampSettledFraction);
        final double alpha = 1.0 - Math.exp(-dtSec / tauSec);

        if (targetPercent > 0.0 && shooterAppliedPercent <= 0.0) {
            shooterAppliedPercent = Math.min(kShooterStartPercentOutput, targetPercent);
        }

        shooterAppliedPercent += (targetPercent - shooterAppliedPercent) * alpha;
        shooterAppliedPercent = Math.max(0.0, Math.min(1.0, shooterAppliedPercent));

        shooterMotor.setControl(shooterRequest.withOutput(shooterAppliedPercent));
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
        if (isAimToHubRequested() && hadVisionPose && lastDistanceToHubMeters > 0) {
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
        final double rangeError = dist - kIdealShootingDistanceMeters;
        double radialSpeed = 0.0;
        if (Math.abs(rangeError) > kRangeDistDeadbandMeters) {
            radialSpeed = MathUtil.clamp(rangeError * kRangeKp,
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

        SmartDashboard.putNumber("3D Target Range Err (m)", rangeError);
        SmartDashboard.putNumber("3D Target Radial Spd", radialSpeed);
        SmartDashboard.putBoolean("3D Target In Range",
            Math.abs(rangeError) <= kRangeDistDeadbandMeters);

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
        return isInputFromOperator(
            driverController.rightBumper().getAsBoolean(),
            isOperatorConnected() && operatorController.rightBumper().getAsBoolean()
        );
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

    private void updateVisionPose() {
        // MegaTag2 pose estimation is handled by CommandSwerveDrivetrain.periodic().
        // Here we just read the fused pose from the drivetrain to compute hub distance.
        updateHubTagFilter();

        // Only trust the pose for aiming/distance if the Limelight has actually seen tags.
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightTableName);
        if (mt2 != null && mt2.tagCount > 0) {
            hadVisionPose = true;
        }

        final Pose2d fusedPose = drivetrain.getState().Pose;
        final Translation2d hubPos = getAllianceHubPosition();
        lastDistanceToHubMeters = fusedPose.getTranslation().getDistance(hubPos);

        field2d.setRobotPose(fusedPose);
        field2d.getObject("Hub").setPose(new Pose2d(hubPos, new Rotation2d()));

        final double aimErrorDeg = computeAimErrorDeg();
        SmartDashboard.putNumber("Hub Distance (m)", lastDistanceToHubMeters);
        SmartDashboard.putNumber("Auto Shooter %", computeDistanceBasedShooterOutput() * 100.0);
        SmartDashboard.putNumber("Aim Error (deg)", aimErrorDeg);
        SmartDashboard.putBoolean("Aimed", Math.abs(aimErrorDeg) < kAimToHubDeadbandDeg);
        SmartDashboard.putBoolean("Vision Valid", hadVisionPose);

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
        final double raw = kShooterDistA + kShooterDistB * Math.sqrt(Math.max(0, lastDistanceToHubMeters));
        return Math.max(kShooterDistMinOutput, Math.min(kShooterDistMaxOutput, raw));
    }

    private double computeAimToHubTurnRate() {
        if (!hadVisionPose) {
            DriverStation.reportWarning("[AIM] No vision data yet, aim-to-hub disabled", false);
            return 0.0;
        }

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

        final double rawTurnRate = kAimToHubKp * errorRad;
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
