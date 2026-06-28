package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret;

@TeleOp(name = "Hood Tuner", group = "Tuning")
public class HoodTunerOpMode extends LinearOpMode {

    private final double[] testDistances = {500, 1000, 1500, 2000, 2500, 3000};
    private int distanceIndex = 0;

    private final double[] savedServo = new double[6];
    private final double[] savedRPM   = new double[6];
    private final boolean[] saved     = new boolean[6];

    private double servoPos   = 0.5;
    private double shooterRPM = 1300.0;

    private final double COARSE     = 0.01;
    private final double FINE       = 0.002;
    private final double RPM_COARSE = 50.0;
    private final double RPM_FINE   = 10.0;

    private boolean prevDpadUp    = false;
    private boolean prevDpadDown  = false;
    private boolean prevDpadLeft  = false;
    private boolean prevDpadRight = false;
    private boolean prevA         = false;
    private boolean prevB         = false;
    private boolean prevY         = false;

    @Override
    public void runOpMode() {
        turret  turretSub  = new turret(hardwareMap);
        shooter shooterSub = new shooter(hardwareMap);

        telemetry.addLine("Hood Tuner ready");
        telemetry.addLine("Place robot at 500mm from target");
        telemetry.addLine("DPAD UP/DOWN   = hood servo");
        telemetry.addLine("DPAD LEFT/RIGHT = shooter RPM");
        telemetry.addLine("LB             = fine adjust");
        telemetry.addLine("A = fire   B = save   Y = redo");
        telemetry.update();

        waitForStart();

        shooterSub.setPower(shooterRPM, false);
        turretSub.servoPos(servoPos);

        while (opModeIsActive()) {

            // All distances done
            if (distanceIndex >= testDistances.length) {
                shooterSub.stopShoot();
                telemetry.addLine("=== TUNING COMPLETE ===");
                telemetry.addLine("Copy into turret.kt:");
                telemetry.addLine("val HOOD_TABLE = listOf(");
                for (int i = 0; i < testDistances.length; i++) {
                    if (saved[i]) {
                        telemetry.addLine("    Pair(" + (int)testDistances[i] + ".0, "
                                + String.format("%.4f", savedServo[i]) + ", "
                                + savedRPM[i] + "),");
                    }
                }
                telemetry.addLine(")");
                telemetry.update();
                sleep(100);
                continue;
            }

            double distance = testDistances[distanceIndex];
            boolean fine    = gamepad1.left_bumper;
            double sStep    = fine ? FINE      : COARSE;
            double rStep    = fine ? RPM_FINE  : RPM_COARSE;

            // Hood servo
            if (gamepad1.dpad_up && !prevDpadUp) {
                servoPos = Math.min(servoPos + sStep, 1.0);
                turretSub.servoPos(servoPos);
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                servoPos = Math.max(servoPos - sStep, 0.0);
                turretSub.servoPos(servoPos);
            }

            // Shooter RPM
            if (gamepad1.dpad_right && !prevDpadRight) {
                shooterRPM += rStep;
                shooterSub.setPower(shooterRPM, false);
            }
            if (gamepad1.dpad_left && !prevDpadLeft) {
                shooterRPM = Math.max(shooterRPM - rStep, 0.0);
                shooterSub.setPower(shooterRPM, false);
            }

            // A = fire
            if (gamepad1.a && !prevA) {
                shooterSub.open();
                sleep(400);
                shooterSub.close();
            }

            // B = save and advance
            if (gamepad1.b && !prevB) {
                savedServo[distanceIndex] = servoPos;
                savedRPM[distanceIndex]   = shooterRPM;
                saved[distanceIndex]      = true;
                distanceIndex++;
            }

            // Y = redo current
            if (gamepad1.y && !prevY) {
                saved[distanceIndex] = false;
            }

            prevDpadUp    = gamepad1.dpad_up;
            prevDpadDown  = gamepad1.dpad_down;
            prevDpadLeft  = gamepad1.dpad_left;
            prevDpadRight = gamepad1.dpad_right;
            prevA         = gamepad1.a;
            prevB         = gamepad1.b;
            prevY         = gamepad1.y;

            // Keep shooter spinning
            shooterSub.setPower(shooterRPM, false);

            // Telemetry
            telemetry.addLine("=== HOOD TUNER ===");
            telemetry.addData("Distance mm",  (int)distance + "  (" + (distanceIndex + 1) + " of " + testDistances.length + ")");
            telemetry.addData("Hood servo",   String.format("%.4f", servoPos));
            telemetry.addData("Shooter RPM",  shooterRPM);
            telemetry.addData("Fine mode LB", fine);
            telemetry.addLine("--- Saved ---");
            for (int i = 0; i < testDistances.length; i++) {
                if (saved[i]) {
                    telemetry.addLine("  " + (int)testDistances[i] + "mm  servo="
                            + String.format("%.4f", savedServo[i])
                            + "  rpm=" + savedRPM[i]);
                }
            }
            telemetry.update();

            sleep(20);
        }

        shooterSub.stopShoot();
    }
}

