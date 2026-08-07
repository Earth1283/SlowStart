package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Team 32008's own TurretTest, copied from FTC-32008 V2 tests/TurretTest.java.
 *
 * ONLY changes: the constants imports point at this repo's kernel.constants,
 * whose classes were renamed to PascalCase (autoConstants -> AutoConstants and
 * so on). No logic touched.
 *
 * NOTE it resolves pedroPathing.Constants to THIS repo's tuned copy, not V2's --
 * that is deliberate: those are the constants measured on this robot.
 */
@TeleOp(name = "V2 TurretTest", group = "32008 V2")
@Configurable
public class TurretTest extends LinearOpMode {
    Shooter shooter = new Shooter();
    public static double targetHeading = 0.0, tor = 0.012;
    JoinedTelemetry joinedTele;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
//        shooter.setPPHeading(0.0);

        waitForStart();

        while (opModeIsActive()) {
//            shooter.turretToDegPP(targetHeading);

            joinedTele.addData("enc", shooter.turret.getCurrentPosition());
//            joinedTele.addData("heading", shooter.getPPHeading());
//            joinedTele.addData("target", targetHeading);
//            joinedTele.addData("diff", targetHeading - shooter.getPPHeading());
            joinedTele.update();
        }
    }
}
