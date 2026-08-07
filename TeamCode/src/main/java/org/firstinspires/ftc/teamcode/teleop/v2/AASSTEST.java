package org.firstinspires.ftc.teamcode.teleop.v2;

import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.BLUE_CLOSE_START;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.RED_CLOSE_START;
import static org.firstinspires.ftc.teamcode.kernel.constants.AutoConstants.TOTAL_SHOOT_TIME;
import static org.firstinspires.ftc.teamcode.kernel.constants.PanelConstants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.SHOOTER_DRIVETRAIN_OFFSET;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.kernel.constants.RobotConstants.teleOpTargetY;
import static org.firstinspires.ftc.teamcode.subsystems.Indicator.Color.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.Indicator.Color.RED;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.targetVelocity;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Team 32008's AASSTEST, with the LOCALIZATION replaced by the autonomous's.
 *
 * The control map, gate logic, trims, indicator and AutoAimSubsystem call pattern
 * are theirs. What changed is where the pose comes from, because that is what was
 * broken.
 *
 * WHY: their version drives the goBILDA Pinpoint directly through Drivetrain and
 * works in the PINPOINT frame. Drivetrain.init() calls pinPoint.resetPosAndIMU(),
 * which is asynchronous and takes a moment; the setPosition() seeding calls that
 * follow it can be swallowed by that reset still completing. The robot then aims
 * from a pose that is wrong by an unknown amount -- confidently, and with no
 * telemetry that looks broken.
 *
 * The autonomous does not have this problem: Pedro's follower owns the Pinpoint,
 * seeds cleanly, and is the localizer whose numbers have actually been verified on
 * this robot. So this now uses the follower, exactly as BlueCloseAuto does:
 *
 *   - pose, field velocity and heading rate all come from the follower
 *   - the shooter offset is applied at the call site, their line
 *   - the goal is in the PEDRO frame, converted once from the kernel's
 *     PINPOINT-frame teleOpTargetX/Y: pedroX = 144 - pinY, pedroY = pinX
 *   - autoAim.update(...) gets the identical argument pattern the auto passes
 *
 * Seeding is now a straight read of autoEndX/Y/H with NO conversion, because the
 * auto publishes those in the Pedro frame. Their file converted; that conversion
 * was only needed because they were feeding a Pinpoint.
 *
 * ALLIANCE STILL COMES FROM THE AUTO -- this never picks a colour, it reads
 * teleOpTargetX/Y which the autonomous writes. Run the matching auto first.
 *
 * DROPPED: their gate-intake heading lock (driveAim's aim=true branch). It was a
 * PINPOINT-frame heading PID inside Drivetrain, which no longer owns the robot.
 * The intakeGate flag is kept for telemetry. Say the word and it can come back as
 * a Pedro-frame lock.
 *
 * FIELD CENTRIC DRIVE. The drivetrain no longer has a front. Push gamepad1's left
 * stick away from you and the robot travels away from you; it does not matter that
 * the auto parks it around 130 deg, or that the driver spins it mid-match. Only the
 * DRIVE is headless -- AutoAim still reads the true heading off the follower for the
 * turret solve, and the right stick still rotates the robot as before.
 *
 * FIELD_FORWARD_DEG = 180 is the drivers' own spec: from (70, 70) stick forward
 * goes to (30, 70) and stick left goes to (70, 30). Confirmed on blue only.
 *
 * If it ever comes out wrong -- other alliance, field flipped, robot re-mounted --
 * run the auto, stand where you drive from, push the left stick straight forward:
 *   robot drives away from you           -> correct, leave it
 *   robot drives toward you              -> add 180
 *   robot drives to your right           -> add 90
 *   robot drives to your left            -> subtract 90
 *   forward is right but left/right are  -> flip INVERT_STRAFE
 *     swapped
 * All of it is Panels, live, no rebuild. ROBOT_CENTRIC = true reverts to the old
 * has-a-front behaviour if field centric ever misbehaves mid-competition.
 *
 * CONTROLS -- unchanged from theirs
 *   gp1 left stick / right stick X  drive       gp1 left bumper   toggle shooter+aim
 *   gp1 right trigger  intake in                gp1 right bumper  FIRE (or gate intake)
 *   gp1 left trigger   intake out               gp2 left bumper   toggle auto turret
 *   dpad up/down  distance trim +/-2            gp2 circle        toggle auto shooter
 *   dpad left/right  turret trim +/-1           gp1 START         re-seed pose
 */
@TeleOp(name = "V2 AASSTEST (real teleop)", group = "32008 V2")
@Configurable
public class AASSTEST extends LinearOpMode {
    Robot robot = new Robot();
    private final AutoAimSubsystem autoAim = new AutoAimSubsystem();
    private Follower follower;

    double targetX = 136.5, targetY = 138;
    int turretTargetHeading = 0;
    double targetATAN;
    double panelPos = 0.806, shooterVelocity = 1600;
    boolean shooterOn = false, autoTurret = true, autoShooter = true, intakeGate = false;
    double distance;
    double angleCorrection = 0, angleCorrectionFar = 0;
    double distanceCorrection = 0, distanceCorrectionFar = -1;
    ElapsedTime timer = new ElapsedTime();
    boolean shootAll = false;
    JoinedTelemetry joinedTele;
    // Initialised, not left null -- their version reads command.targetDist before
    // the first assignment is guaranteed.
    AutoAimSubsystem.TurretCommand command = new AutoAimSubsystem.TurretCommand();
    boolean lastFar = false;

    /** Goal in the PEDRO frame, converted from the kernel's PINPOINT-frame pair. */
    private double goalX, goalY;

    public static double STICK_DEADBAND = 0.05;
    public static double TURN_SCALE = 0.9;

    /**
     * FALSE = FIELD CENTRIC: push the stick away from you and the robot drives away
     * from you, whatever direction its nose happens to be pointing.
     *
     * Pedro, VectorCalculator.setTeleOpMovementVectors:
     *
     *     teleopDriveVector.setOrthogonalComponents(forward, lateral);
     *     if (robotCentric) teleopDriveVector.rotateVector(currentPose.getHeading());
     *     teleopDriveVector.rotateVector(headingOffset);
     *
     * The vector is consumed in the FIELD frame downstream. Rotating it by the robot
     * heading is what makes it robot-relative, so robotCentric = true was genuinely
     * robot-centric -- and that is exactly the "it still has a front and a back"
     * behaviour we are getting rid of. With it false the stick vector goes into the
     * field frame untouched, and FIELD_FORWARD_DEG aims it.
     *
     * Left true as a live escape hatch: flip it in Panels and the old robot-centric
     * feel comes straight back, with the offset suppressed so the two cannot stack.
     */
    public static boolean ROBOT_CENTRIC = false;

    /**
     * THE ONE KNOB. Pedro-frame compass heading, in degrees, that the robot drives
     * when gamepad1's left stick is pushed straight forward. Stick left goes 90 deg
     * counter-clockwise of it, and so on.
     *
     * 180 is the behaviour the drivers asked for, stated as field coordinates:
     *
     *   from (70, 70), stick forward -> (30, 70)    X falls, Y held  =  field -X
     *   from (70, 70), stick left    -> (70, 30)    Y falls, X held  =  field -Y
     *
     * Those two are consistent with each other -- -Y is exactly 90 deg counter-
     * clockwise of -X, which is where Pedro puts "left" -- so one rotation covers
     * both and INVERT_STRAFE stays off. Drivers are therefore behind the X = 144
     * wall looking toward -X.
     *
     * SET FROM THE BLUE SIDE. Only Blue autos exist, so only blue is confirmed. If
     * the red alliance station sits on the opposite wall, red wants 0 here; if it is
     * beside blue on the same wall, 180 covers both. Drive it once on red and see.
     * Same Panels field either way, no rebuild.
     */
    public static double FIELD_FORWARD_DEG = 180.0;

    // Live axis flips, so a wrong sign can be fixed from Panels instead of a rebuild.
    public static boolean INVERT_FORWARD = false;
    public static boolean INVERT_STRAFE = false;
    public static boolean INVERT_TURN = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Pedro's follower owns the drive motors AND the Pinpoint, so Drivetrain
        // must NOT be initialised -- two owners of the same hardware fight.
        // This is robot.init(hardwareMap, true) minus the drivetrain.
        robot.intake.init(hardwareMap);
        robot.shooter.init(hardwareMap, true);   // aass = true: AutoAim owns turret + hood
        robot.indicator.init(hardwareMap);
        autoAim.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        // Pedro frame in, Pedro frame out -- no conversion, unlike their Pinpoint seed.
        follower.setStartingPose(new Pose(autoEndX, autoEndY, autoEndH));

        targetX = teleOpTargetX;
        targetY = teleOpTargetY;
        goalX = 144.0 - teleOpTargetY;
        goalY = teleOpTargetX;

        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        while (opModeInInit()) {
            follower.update();
            joinedTele.addData("aim", "AutoAimSubsystem via Pedro follower");
            joinedTele.addData("drive", ROBOT_CENTRIC ? "ROBOT CENTRIC" : "FIELD CENTRIC");
            joinedTele.addData("stick fwd drives (deg)", ROBOT_CENTRIC ? "robot nose" : FIELD_FORWARD_DEG);
            joinedTele.addData("goal X (pedro)", goalX);
            joinedTele.addData("goal Y (pedro)", goalY);
            // Check these against where the robot actually is. Stale if no auto ran.
            joinedTele.addData("seeded x", follower.getPose().getX());
            joinedTele.addData("seeded y", follower.getPose().getY());
            joinedTele.addData("seeded h", Math.toDegrees(follower.getPose().getHeading()));
            joinedTele.addData("turret ticks", autoAim.getCurrentTick());
            joinedTele.update();
        }
        if (isStopRequested()) return;

        follower.startTeleopDrive(true);

        while (opModeIsActive()) {
            follower.update();

            Pose p = follower.getPose();
            Vector v = follower.getVelocity();
            double headingDeg = Math.toDegrees(p.getHeading());
            // Their shooter offset, applied at the call site exactly as in the auto.
            double robotX = p.getX() + Math.cos(p.getHeading()) * SHOOTER_DRIVETRAIN_OFFSET;
            double robotY = p.getY() + Math.sin(p.getHeading()) * SHOOTER_DRIVETRAIN_OFFSET;
            double vx = v.getXComponent();
            double vy = v.getYComponent();
            // Pedro reports heading rate in RADIANS/sec; AutoAim wants degrees.
            double omegaDeg = Math.toDegrees(follower.getAngularVelocity());
            boolean isBraking = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) < 0.15;

            if (gamepad1.start) {
                follower.setPose(targetY > 50 ? BLUE_CLOSE_START.copy() : RED_CLOSE_START.copy());
            }

            if (shooterOn) {
                if (command.targetDist > 120) {
                    AutoAimSubsystem.SHOT_DISTANCE_OFFSET = distanceCorrectionFar;
                    command = autoAim.update(
                            robotX, robotY, vx, vy,
                            headingDeg, omegaDeg,
                            goalX, goalY,
                            !autoTurret, 60.0 + distanceCorrectionFar,
                            true, isBraking, angleCorrectionFar
                    );
                } else {
                    AutoAimSubsystem.SHOT_DISTANCE_OFFSET = distanceCorrection;
                    command = autoAim.update(
                            robotX, robotY, vx, vy,
                            headingDeg, omegaDeg,
                            goalX, goalY,
                            !autoTurret, 60.0 + distanceCorrection,
                            true, isBraking, angleCorrection
                    );
                }
            } else {
                // Their turret park: a target at (-1,0) from (0,0) solves past the
                // safe-angle limit, so AutoAim recentres and reports no target.
                command = autoAim.update(0, 0, 0, 0, 0, 0, -1, 0, false, 0, true, true, 0);
            }
            targetATAN = command.targetTurretAngle;
            distance = command.targetDist;

            drive();

            if (gamepad1.right_trigger > 0) {
                robot.intake.intakeIn();
                shooterOn = false;
            } else if (gamepad1.left_trigger > 0) {
                robot.intake.intakeOut();
            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                if (shooterOn) {
                    if (autoShooter) {
                        robot.intake.intakeIn(robot.shooter.calculateIntakePower());
                    } else {
                        robot.intake.intakeIn(INTAKE_POWER);
                    }
                    lastFar = command.targetDist > 120;
                } else {
                    robot.intake.intakeIn();
                    intakeGate = true;
                    shooterOn = false;
                }
            } else if (gamepad1.rightBumperWasReleased()) {
                intakeGate = false;
            } else if (!shootAll) {
                robot.intake.intakeStop();
            }

            if (shootAll && timer.milliseconds() > TOTAL_SHOOT_TIME) {
                shootAll = false;
            }

            if (gamepad1.leftBumperWasPressed()) {
                shooterOn = !shooterOn;
            }

            if (shooterOn && command.hasTarget) {
                robot.intake.gateOpen();
                targetVelocity = command.targetRpm;
                robot.shooter.setShooterVelocity(targetVelocity);
                robot.indicator.setColor(GREEN);
            } else {
                robot.intake.gateClose();
                robot.intake.intakeDisengage();
                robot.shooter.shooterHold();
                robot.indicator.setColor(RED);
            }

            if (gamepad2.leftBumperWasPressed()) autoTurret = !autoTurret;
            if (gamepad2.circleWasPressed()) {
                autoShooter = !autoShooter;
                if (!autoShooter) {
                    shooterVelocity = 1400;
                    panelPos = 0.37;
                    INTAKE_POWER = 0.95;
                }
            }

            if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
                if (autoShooter) {
                    if (lastFar) distanceCorrectionFar += 2;
                    else distanceCorrection += 2;
                } else shooterVelocity += 20;
            }
            if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
                if (autoShooter) {
                    if (lastFar) distanceCorrectionFar -= 2;
                    else distanceCorrection -= 2;
                } else shooterVelocity -= 20;
            }
            if (gamepad2.triangle && !autoShooter) panelPos += 0.005;
            if (gamepad2.cross && !autoShooter) panelPos -= 0.005;
            if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
                if (lastFar) angleCorrectionFar += 1;
                else angleCorrection += 1;
            }
            if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
                if (lastFar) angleCorrectionFar -= 1;
                else angleCorrection -= 1;
            }

            report();
        }
        robot.intake.intakeStop();
        robot.intake.gateClose();
        robot.shooter.shooterStop();
        autoAim.stop();
    }

    private double deadband(double x) {
        return Math.abs(x) < STICK_DEADBAND ? 0.0 : x;
    }

    /**
     * Field centric through the follower.
     *
     * Sign convention, from Pedro's VectorCalculator: setOrthogonalComponents takes
     * (forward, lateral) with lateral positive to the LEFT, and rotateVector adds to
     * the vector angle, so positive is counter-clockwise. Stick left is negative
     * left_stick_x, so lateral negates it; stick right is positive right_stick_x and
     * should turn clockwise, so turn negates it too.
     *
     * With ROBOT_CENTRIC false the (forward, lateral) pair starts out along the field
     * +X axis and FIELD_FORWARD_DEG rotates it to wherever the drivers are facing.
     * The offset is forced to zero when ROBOT_CENTRIC is true, because Pedro applies
     * the heading rotation AND the offset -- leaving the offset in would give
     * robot-centric driving skewed by 90 deg, which is neither mode.
     *
     * TURN IS UNAFFECTED BY EITHER MODE. Pedro builds the heading vector separately
     * (teleopHeadingVector), so the right stick still spins the robot the same way it
     * always did.
     */
    private void drive() {
        double forward = deadband(-gamepad1.left_stick_y);
        double strafe  = deadband(-gamepad1.left_stick_x);
        double turn    = deadband(-gamepad1.right_stick_x) * TURN_SCALE;

        if (INVERT_FORWARD) forward = -forward;
        if (INVERT_STRAFE)  strafe  = -strafe;
        if (INVERT_TURN)    turn    = -turn;

        follower.setTeleOpDrive(forward, strafe, turn, ROBOT_CENTRIC,
                ROBOT_CENTRIC ? 0.0 : Math.toRadians(FIELD_FORWARD_DEG));
    }

    private void report() {
        joinedTele.addData("x", follower.getPose().getX());
        joinedTele.addData("y", follower.getPose().getY());
        joinedTele.addData("h", Math.toDegrees(follower.getPose().getHeading()));
        joinedTele.addData("drive", ROBOT_CENTRIC ? "ROBOT CENTRIC" : "FIELD CENTRIC");
        joinedTele.addData("fieldForward", FIELD_FORWARD_DEG);
        joinedTele.addData("shooterOn", shooterOn);
        joinedTele.addData("autoTurret", autoTurret);
        joinedTele.addData("autoShooter", autoShooter);
        joinedTele.addData("intakeGate", intakeGate);
        joinedTele.addData("hasTarget", command.hasTarget);
        joinedTele.addData("aimLocked", command.isAimLocked);
        joinedTele.addData("targetTurret", command.targetTurretAngle);
        joinedTele.addData("aimError", command.aimError);
        joinedTele.addData("targetDist", distance);
        joinedTele.addData("turret", autoAim.getCurrentTick());
        joinedTele.addData("turretDeg", autoAim.getCurrentTurretAngle());
        joinedTele.addData("shooterT", targetVelocity);
        joinedTele.addData("shooterVL", robot.shooter.leftShooter.getVelocity());
        joinedTele.addData("shooterVR", robot.shooter.rightShooter.getVelocity());
        joinedTele.addData("hoodActPos", autoAim.hood.getPosition());
        joinedTele.addData("angleCorrection", angleCorrection);
        joinedTele.addData("distanceCorrection", distanceCorrection);
        joinedTele.addData("angleCorrectionFar", angleCorrectionFar);
        joinedTele.addData("distanceCorrectionFar", distanceCorrectionFar);
        joinedTele.update();
    }
}
