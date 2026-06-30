package org.firstinspires.ftc.teamcode.opmode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret;

/**
 * Encoder-based mecanum autonomous for SWYFT Drive V2 modules.
 * No path-following library — pure encoder RUN_TO_POSITION moves.
 *
 * Hardware config names expected (rename to match your config):
 *   "lF", "rF", "lR", "rR"
 *
 * SWYFT Drive V2 specs used for the math below:
 *   Motor encoder: 28 PPR -- this is ALREADY the decoded counts/rev, do not x4 again
 *   Gear ratio:    12.7:1
 *   -> 28 * 12.7 = 355.6 ticks per WHEEL revolution
 *   Wheel diameter: 86 mm = 3.386 in -> circumference = 10.638 in
 *   -> TICKS_PER_INCH ≈ 33.4
 */
@Autonomous(name = "6 Ball Blue Gate")
public class SixBallBlueGate extends LinearOpMode {

    // ---- SWYFT Drive V2 constants ----
    // NOTE: SWYFT/goBILDA-style motors spec "28 PPR" as the ALREADY-DECODED
    // counts per motor revolution (quadrature decoding included in the spec).
    // Do NOT multiply by 4 again -- that was causing a 4x overshoot on distance.
    static final double MOTOR_TICKS_PER_REV = 28;            // already decoded CPR
    static final double GEAR_RATIO          = 12.7;         // SWYFT V2 gearbox
    static final double WHEEL_DIAMETER_IN   = 3.386;        // 86mm wheel
    static final double TICKS_PER_WHEEL_REV = MOTOR_TICKS_PER_REV * GEAR_RATIO; // 355.6
    static final double WHEEL_CIRCUMFERENCE_IN = Math.PI * WHEEL_DIAMETER_IN;   // 10.638 in
    static final double TICKS_PER_INCH      = TICKS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_IN; // ~33.4

    // Strafing has more wheel slip than forward driving — this empirical
    // multiplier compensates. Start at 1.1 and tune on your actual robot.
    static final double STRAFE_CORRECTION   = 1.1;

    // Track width / wheelbase (in inches) -- measure your chassis and set these
    // for the turnDegrees() math. These are placeholders; SWYFT module footprint
    // is small but your chassis width depends on your frame build.
    static final double TRACK_WIDTH_IN      = 14.0;  // distance between left/right wheel centers
    static final double WHEEL_BASE_IN       = 14.0;  // distance between front/back wheel centers

    static final double DEFAULT_POWER       = 0.5;

    DcMotorEx leftFront, rightFront, leftRear, rightRear;

    // Subsystems
    intake intake;
    shooter shooter;
    turret turret;

    @Override
    public void runOpMode() {
        leftFront  = hardwareMap.get(DcMotorEx.class, "lF");
        rightFront = hardwareMap.get(DcMotorEx.class, "rF");
        leftRear   = hardwareMap.get(DcMotorEx.class, "lR");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rR");

        // Matches directions from your other auto
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{leftFront, rightFront, leftRear, rightRear}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Subsystems — construct once here, same as the drive motors above
        intake  = new intake(hardwareMap);
        shooter = new shooter(hardwareMap);
        turret  = new turret(hardwareMap);
        turret.resetEncoder();

        waitForStart();
        if (isStopRequested()) return;

        // ===================================================================
        // EDIT THIS SECTION to match the path from your planner.
        // drive(forwardIn, strafeIn, power)  -> forward(+)/back(-), right(+)/left(-) strafe
        // turn(degrees, power)               -> positive = clockwise
        // intake/shooter/turret calls go here too, in whatever order you need them
        //
        // Example sequence placeholder — replace with your actual waypoints:
        // ===================================================================
        double rpm = turret.aimForDistance(1000.0);
        shooter.setPower(rpm, false);
        drive(-24, 0, DEFAULT_POWER);

        sleep(2000);

        turret.track(-6.0, 0.0);
        sleep(1000);
        shooter.toggleGate();
        intake.intake();
        sleep(400);
        intake.stopIntake();
        sleep(1000);
        intake.intake();
        sleep(300);
        intake.stopIntake();
        sleep(1000);
        intake.intake();
        sleep(800);
        intake.stopIntake();
        shooter.toggleGate();


        turn(110,DEFAULT_POWER);

        drive(24,0,DEFAULT_POWER);

        turn(-80,DEFAULT_POWER);

        intake.intake();

        drive(24,0,0.3 );

        sleep(200);

        intake.stopIntake();

        drive(-5,0,DEFAULT_POWER);

        turn(-100,DEFAULT_POWER);

        shooter.setPower(rpm, false);

        drive(33,0,DEFAULT_POWER);

        turret.track(70.0, 0.0);
        sleep(600);
        shooter.toggleGate();
        intake.intake();
        sleep(400);
        intake.stopIntake();
        sleep(1000);
        intake.intake();
        sleep(300);
        intake.stopIntake();
        sleep(1000);
        intake.intake();
        sleep(800);
        intake.stopIntake();
        shooter.toggleGate();

        turret.track(0.1, 0.0);

        turn(25,DEFAULT_POWER);

        drive(-37,0,DEFAULT_POWER);

        turn(90,DEFAULT_POWER);

        drive(25,0,DEFAULT_POWER);

        sleep(1000);

        drive(-10,0,DEFAULT_POWER);






        // add more drive()/turn()/subsystem calls here to match your full path
    }

    /** Drive using mecanum kinematics with encoder RUN_TO_POSITION. */
    void drive(double forwardIn, double strafeIn, double power) {
        double fwdTicks = forwardIn * TICKS_PER_INCH;
        double strTicks = strafeIn * TICKS_PER_INCH * STRAFE_CORRECTION;

        int lfTarget = (int) Math.round(fwdTicks + strTicks);
        int rfTarget = (int) Math.round(fwdTicks - strTicks);
        int lrTarget = (int) Math.round(fwdTicks - strTicks);
        int rrTarget = (int) Math.round(fwdTicks + strTicks);

        runToPosition(lfTarget, rfTarget, lrTarget, rrTarget, power);
    }

    /** Turn in place using encoders (tank-style differential on a mecanum chassis). */
    void turn(double degrees, double power) {
        double wheelRadius = Math.hypot(TRACK_WIDTH_IN, WHEEL_BASE_IN) / 2.0;
        double arcLength = Math.toRadians(degrees) * wheelRadius;
        int ticks = (int) Math.round(arcLength * TICKS_PER_INCH);

        // Clockwise turn: right side back, left side forward
        runToPosition(ticks, -ticks, ticks, -ticks, power);
    }

    private void runToPosition(int lf, int rf, int lr, int rr, double power) {
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + lf);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + rf);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + lr);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + rr);

        for (DcMotorEx m : new DcMotorEx[]{leftFront, rightFront, leftRear, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftFront.setPower(Math.abs(power));
        rightFront.setPower(Math.abs(power));
        leftRear.setPower(Math.abs(power));
        rightRear.setPower(Math.abs(power));

        while (opModeIsActive() &&
                leftFront.isBusy() && rightFront.isBusy() &&
                leftRear.isBusy() && rightRear.isBusy()) {
            // Optional: add telemetry here to monitor progress
            idle();
        }

        for (DcMotorEx m : new DcMotorEx[]{leftFront, rightFront, leftRear, rightRear}) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}