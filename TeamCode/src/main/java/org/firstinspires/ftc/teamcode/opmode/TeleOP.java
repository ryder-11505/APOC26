package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.config.pedro.Constants.createFollower;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.config.staticData.PoseStorage;


@TeleOp(group = "advanced", name = "Teleop")
public class TeleOP extends LinearOpMode {
    public static double SlowmodeSpeed   = 0.5;
    public static double SlowmodeTurning = 0.5;
    public static double TriggerMin      = 0.01;

    // Depth of the goal, front face to back face (where Dr/Db are measured to).
    // The shooter RPM/hood table was tuned against distance-to-front-face,
    // so this is subtracted before the table lookup.
    public static double GoalDepthMM = 19.0 * 25.4; // 355.6 mm

    // Turret/shooter launch point offset from robot center of rotation,
    // along the chassis forward axis. Negative = behind center.
    public static double ShooterForwardOffsetInches = -4.3;

    public int targetId = 24;

    private boolean isScanning = false;

    private TelemetryManager telemetryM;
    private Follower follower;
    public static Pose startingPose;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake intake       = new intake(hardwareMap);
        shooter outtake     = new shooter(hardwareMap);
        turret spinSimple   = new turret(hardwareMap);
        follower            = createFollower(hardwareMap);

        PoseStorage.splitControls = true;

        while (!isStarted()) {
            follower.setStartingPose(startingPose == null ? new Pose(0, 0, 0) : startingPose);
            follower.update();
            spinSimple.resetEncoder();

            if (gamepad1.b) {
                PoseStorage.isRedAlliance = true;
                targetId = 24;
            } else if (gamepad1.x) {
                PoseStorage.isRedAlliance = false;
                targetId = 20;
            }
        }

        if (isStopRequested()) return;

        follower.startTeleopDrive();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        double offset = 0.0;
        double pos    = 0.0;

        PoseStorage.isInit = false;
        Deadline matchTimer    = new Deadline(2, TimeUnit.MINUTES);
        Deadline transferTimer = new Deadline(500, TimeUnit.MILLISECONDS);

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    true
            );

            double X = follower.getPose().getX();
            double Y = follower.getPose().getY();
            double H = follower.getPose().getHeading();

            // Actual launch point of the shooter: the turret pivot is fixed
            // to the chassis ShooterForwardOffsetInches along the forward
            // axis, so it rotates with chassis heading rather than staying
            // at the robot's center of rotation.
            double launchX = X + ShooterForwardOffsetInches * Math.cos(H);
            double launchY = Y + ShooterForwardOffsetInches * Math.sin(H);

            // Distance to each goal in inches, measured from the actual
            // launch point rather than the robot's center of rotation.
            double Dr = Math.sqrt(((72.0 - launchY) * (72.0 - launchY)) + ((72.0 + launchX) * (72.0 + launchX)));
            double Db = Math.sqrt(((-72.0 - launchY) * (-72.0 - launchY)) + ((72.0 + launchX) * (72.0 + launchX)));

            // Turret heading angles
            double Hr2 = (-H) - Math.acos((72.0 + launchX) / Dr);
            double Hb2 = (-H) + Math.acos((72.0 + launchX) / Db);

            // Distance in mm for the tuned lookup table (1 inch = 25.4 mm)
            // Dr/Db are measured to the back of the goal, but the hood/RPM
            // table was tuned to the front face, so correct for goal depth
            // before doing the table lookup.
            double distMM_R = Dr * 25.4;
            double distMM_B = Db * 25.4;
            double tableDistMM_R = distMM_R - GoalDepthMM;
            double tableDistMM_B = distMM_B - GoalDepthMM;

            double finalTargetRPM = 0.0;

            double boost = 1.0;

            // Boost when intake is running (balls moving through the system)
            boolean shouldBoost = gamepad1.a || gamepad2.a;

            // Right bumper = aim and spin up using tuned table
            if (PoseStorage.isRedAlliance) {
                if (gamepad1.dpadLeftWasPressed())  offset += 2.0;
                if (gamepad1.dpadRightWasPressed()) offset -= 2.0;
                if (gamepad1.dpadUpWasPressed()) boost += 0.1;
                if (gamepad1.dpadUpWasPressed()) boost -= 0.1;
                spinSimple.track(Math.toDegrees(Hr2), offset);
                finalTargetRPM = spinSimple.aimForDistance(tableDistMM_R) * boost;
                outtake.setPower(finalTargetRPM, shouldBoost);

            } else if (!PoseStorage.isRedAlliance) {
                if (gamepad1.dpadLeftWasPressed())  offset += 2.0;
                if (gamepad1.dpadRightWasPressed()) offset -= 2.0;
                if (gamepad1.dpadUpWasPressed()) boost += 0.1;
                if (gamepad1.dpadUpWasPressed()) boost -= 0.1;
                spinSimple.track(Math.toDegrees(Hb2), offset);
                finalTargetRPM = spinSimple.aimForDistance(tableDistMM_B) * boost;
                outtake.setPower(finalTargetRPM, shouldBoost);
            }

            if (gamepad1.rightBumperWasReleased()) {
                spinSimple.track(0.0, 0.0);
                spinSimple.servoPos(1.0);
                outtake.stopShoot();
                offset = 0.0;
            }

            // Manual servo trim via gamepad2
            if (gamepad2.dpadUpWasPressed()) {
                spinSimple.servoPos(pos += 0.05);
                telemetry.addLine("servo up");
            }
            if (gamepad2.dpadDownWasPressed()) {
                spinSimple.servoPos(pos -= 0.05);
                telemetry.addLine("servo down");
            }


            PoseStorage.shouldHallucinate = (PoseStorage.splitControls ? gamepad2 : gamepad1).guide;

            // Intake
            if (gamepad1.x) {
                intake.outake();
                telemetry.addLine("x");
            }
            if (gamepad1.xWasReleased()) {
                intake.stopIntake();
            }
            if (gamepad1.a || gamepad2.a) {
                intake.intake();
                telemetry.addLine("a");
            }
            if (gamepad1.aWasReleased() || gamepad2.aWasReleased()) {
                intake.stopIntake();
            }

            // Gate toggle
            if (gamepad1.leftBumperWasPressed()) {
                outtake.toggleGate();
            }

            // Telemetry
            telemetry.addData("Intended RPM",               finalTargetRPM);
            telemetry.addData("Left Motor Velocity",        outtake.getMotor().getVelocity());
            telemetry.addData("Right Motor Velocity",       outtake.getMotor2().getVelocity());
            telemetry.addData("Red distance (in)",          Dr);
            telemetry.addData("Blue distance (in)",         Db);
            telemetry.addData("Red distance (mm)",          distMM_R);
            telemetry.addData("Blue distance (mm)",         distMM_B);
            telemetry.addData("Red table dist (mm)",        tableDistMM_R);
            telemetry.addData("Blue table dist (mm)",       tableDistMM_B);
            telemetry.addData("Servo pos",                  spinSimple.getServo().getPosition());
            telemetry.addData("Red angle (deg)",            Math.toDegrees(Hr2));
            telemetry.addData("Blue angle (deg)",           Math.toDegrees(Hb2));
            telemetry.addData("X (in)",                     X);
            telemetry.addData("Y (in)",                     Y);
            telemetry.addData("Heading (deg)",              Math.toDegrees(H));
            telemetry.addData("Spin Target",                spinSimple.getSpinTargetPos());
            telemetry.addData("Spin Pos (deg)",             spinSimple.getSpinCurrentPosition() / (144.0 / 45.0));
            telemetry.addData("Spin Pos (ticks)",           spinSimple.getSpinCurrentPosition());
            telemetry.addData("Spin Busy",                  spinSimple.getSpinMotor().isBusy());
            telemetry.addData("Spin Power",                 spinSimple.getSpinMotor().getPower());
            telemetry.addData("Team",                       PoseStorage.isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Hallucinating",              PoseStorage.shouldHallucinate);

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            telemetry.update();
            telemetryM.update();
        }
    }

    enum FinishingState { Intake, Outtake, None }
    enum Motif { PPG, PGP, GPP, None }
}