package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.config.pedro.Constants.createFollower;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
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
    public static double SlowmodeSpeed = 0.5;
    public static double SlowmodeTurning = 0.5;

    public static double TriggerMin = 0.01;

    public int targetId = 24; // default tag ID

    private boolean isScanning = false;

    private TelemetryManager telemetryM;

    private Follower follower;
    public static Pose startingPose;



    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake intake = new intake(hardwareMap);
        shooter outtake = new shooter(hardwareMap);
        turret spinSimple = new turret(hardwareMap);
        follower = createFollower(hardwareMap);


        Pose pose = follower.getPose();

        PoseStorage.splitControls = true;
        while (!isStarted()) {
            follower.setStartingPose(startingPose == null ? new Pose(0,0,0) : startingPose);
            follower.update();
            spinSimple.resetEncoder();
//            telemetryM.update();


//            if (gamepad2.back) {
//                PoseStorage.splitControls = true;
//            } else if (gamepad1.back) {
//                PoseStorage.splitControls = false;
//            }

            if (gamepad1.b) {
                PoseStorage.isRedAlliance = true; // Tag ID 24
                targetId = 24;
            } else if (gamepad1.x) {
                PoseStorage.isRedAlliance = false; // Tag ID 20
                targetId = 20;
            }
        }

        if (isStopRequested()) return;

        follower.startTeleopDrive();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


//        LoggableAction finishingAction = new Loggable("INIT", new ParallelAction(
//                new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        spinSimple.track(0.0, 0.0);
//                        return false;
//                    }
//                }
//        ));


        double angle = 25.0;

        double offset = 0.0;

        double pos = 0.0;

        PoseStorage.isInit = false;
        Deadline matchTimer = new Deadline(2, TimeUnit.MINUTES);
        Deadline transferTimer = new Deadline(500, TimeUnit.MILLISECONDS);
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            follower.setTeleOpDrive(
                     -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    true // Robot Centric
            );

            double X = follower.getPose().getX();

            double Y = follower.getPose().getY();

            double H = follower.getPose().getHeading();

            double Dr = Math.sqrt(((72.0 - Y) * (72.0 - Y)) + ((72.0 + X) * (72.0 + X)));

            double Db = Math.sqrt(((-72.0 - Y) * (-72.0 - Y)) + ((72.0 + X) * (72.0 + X)));

            double Hr = Math.atan((72.0 + X) / (72.0 - Y)) + Math.toRadians(90.0) - H;

            double Hb = Math.atan((72.0 + X) / (72.0 + Y)) + Math.toRadians(180.0) - H;

            double Hr2 = (- H) - Math.acos((72.0 + X) / Dr);

            double Hb2 = (- H) + Math.acos((72.0 + X) / Db);

            double Ab = 0.0549451 * Db + 28.17582;

            double Ar = 0.0549451 * Dr + 28.17582;

            double viB = (Db / Math.cos((90 - Ab))) * Math.sqrt((386.4 / (2 * (Db * Math.tan((Math.toRadians(90 - Ab))) - 20.75))));

            double viR = (Dr / Math.cos((90 - Ar))) * Math.sqrt((386.4 / (2 * (Dr * Math.tan((Math.toRadians(90 - Ar))) - 20.75))));

            double rpmB = (120 * viB) / (Math.PI * 2.83465 * 1.2);

            double rpmR = (120 * viR) / (Math.PI * 2.83465 * 1.2);


//            if (gamepad1.b && PoseStorage.isRedAlliance) {
//                outtake.setPower(rpmR);
//                telemetry.addLine("b");
//            } else if (gamepad1.b && !PoseStorage.isRedAlliance) {
//                outtake.setPower(rpmB);
//                telemetry.addLine("b");
//            }


            double finalTargetRPM = 0.0;

            if (gamepad1.b) {
                if (PoseStorage.isRedAlliance) {
                    // If the math defaults to 0 because of a dead zone, fall back to your preset long/short speeds
                    if (rpmR <= 50.0) {
                        finalTargetRPM = 990.0; // Your shooter.speedLong default fallback
                    } else {
                        finalTargetRPM = rpmR;
                    }
                } else {
                    if (rpmB <= 50.0) {
                        finalTargetRPM = 990.0; // Fallback
                    } else {
                        finalTargetRPM = rpmB;
                    }
                }

                // Command the unthrottled target power safely
                outtake.setPower(finalTargetRPM);
            }

            if (gamepad1.bWasReleased()) {
                outtake.stopShoot();
            }

            if (PoseStorage.isRedAlliance) {
                if (rpmR <= 50.0) {
                    finalTargetRPM = 600.0; // Lower this fallback number if the "dead zone" speed is too intense
                } else {
                    finalTargetRPM = rpmR;
                }
            } else if (!PoseStorage.isRedAlliance) {
                if (rpmB <= 50.0) {
                    finalTargetRPM = 600.0; // Lower this fallback number if the "dead zone" speed is too intense
                } else {
                    finalTargetRPM = rpmB;
                }
            }

            if (gamepad1.right_bumper && PoseStorage.isRedAlliance) {
                if (gamepad1.dpadLeftWasPressed()) {
                    offset += 2.0;
                } else if (gamepad1.dpadRightWasPressed()) {
                    offset -= 2.0;
                }
                spinSimple.track(Math.toDegrees(Hr2), offset);
                spinSimple.hoodAngle(Ar);
            } else if (gamepad1.right_bumper && !PoseStorage.isRedAlliance) {
                if (gamepad1.dpadLeftWasPressed()) {
                    offset += 2.0;
                } else if (gamepad1.dpadRightWasPressed()) {
                    offset -= 2.0;
                }
                spinSimple.track(Math.toDegrees(Hb2), offset);
                spinSimple.hoodAngle(Ab);
            }

            if (gamepad1.rightBumperWasReleased()) {
                spinSimple.track(0.0, 0.0);
                spinSimple.hoodAngle(10.0);
                offset = 0.0;
            }



            if (gamepad2.dpadUpWasPressed()) {
                spinSimple.servoPos(pos += 0.05);
                telemetry.addLine("servo up");
            }


            if (gamepad2.dpadDownWasPressed()) {
                spinSimple.servoPos(pos -= 0.05);
                telemetry.addLine("servo down");
            }



//            if (gamepad2.back) {
//                PoseStorage.splitControls = true;
//            } else if (gamepad1.back) {
//                PoseStorage.splitControls = false;
//            }

//            Vector2d input = new Vector2d(
//                    gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x
//            );

            PoseStorage.shouldHallucinate = (PoseStorage.splitControls ? gamepad2 : gamepad1).guide;


//            if (gamepad1.right_bumper) {
//                PoseStorage.isRedAlliance = true; // Tag ID 24
//            } else if (gamepad1.left_bumper) {
//                PoseStorage.isRedAlliance = false; // Tag ID 20
//            }

//            slowMode.update(gamepad1.y);
//            fieldMode.update(gamepad1.x);



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


            if (gamepad1.left_bumper) {
                outtake.open();
                telemetry.addLine("left_bumper");
            }

            if (gamepad1.leftBumperWasReleased()) {
                outtake.close();
            }

//            input = input.times(slowMode.val ? SlowmodeSpeed : 1);
//
//            if (fieldMode.val) {
//                input = poseEstimate.heading.inverse().plus((PoseStorage.isRedAlliance ? 1 : -1) * Math.PI / 2).times(input);
//            }
//
//            PoseVelocity2d drivePower = new PoseVelocity2d(
//                    input,
//                    -gamepad1.right_stick_x * (slowMode.val ? SlowmodeTurning : 1)
//            );
//
//            Logging.DEBUG("X Input", input.x);
//            Logging.DEBUG("Y Input", input.y);

//            // Pass in the rotated input + right stick value for rotation
//            // Rotation is not part of the rotated input thus must be passed in separately
//            driveBase.setDrivePowers(drivePower);


//            Logging.LOG("FINISH_STATE", finishState);
//            if (finishingAction != null) {
//                Logging.DEBUG("FINISH_ACTION", finishingAction.getName());
//
//                if (!finishingAction.run(p)) {
//                    Logging.LOG("FINISH_ACTION_FINISHED");
//                    finishingAction = null;
//                    finishState = FinishingState.None;
//                }
//            }
//
//            Logging.LOG("SCANNING_MODE", isScanning);
//            if (isScanning) {
//                Logging.LOG("SCAN_STATE", scanningAction != null ? "SCANNING" : "TRACKING");
//            }
//            pinpoint.update();
//            pose = pinpoint.getPosition();
//            Logging.LOG("X coordinate (IN)T", pose.getX(DistanceUnit.INCH));
//            Logging.LOG("Y coordinate (IN)T", pose.getY(DistanceUnit.INCH));
//            Logging.LOG("Heading angle (DEGREES)T", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("1. Intended Target RPM", finalTargetRPM);
            telemetry.addData("2. Left Motor Velocity (outL)", outtake.getMotor().getVelocity());
            telemetry.addData("3. Right Motor Velocity (outR)", outtake.getMotor2().getVelocity()); // Add getMotor2() helper to shooter.kt
            telemetry.addData("target speed blue", viB);
            telemetry.addData("target rpm blue", rpmB);
            telemetry.addData("target speed red", viR);
            telemetry.addData("target rpm red", rpmR);
            telemetry.addData("target servo angle blue", Ab);
            telemetry.addData("target servo pos blue", -0.03125 * Ab + 1.9375);
            telemetry.addData("target servo angle red", Ar);
            telemetry.addData("target servo pos red", -0.03125 * Ar + 1.9375);
            telemetry.addData("servo pos", spinSimple.getServo().getPosition());
            telemetry.addData("Blue angle (deg)", Math.toDegrees(Hb2));
            telemetry.addData("Red angle (deg)", Math.toDegrees(Hr2));
            telemetry.addData("Blue distance (IN)", Db);
            telemetry.addData("Red distance (IN)", Dr);
            telemetry.addData("X coordinate (IN)", follower.getPose().getX());
            telemetry.addData("Y coordinate (IN)", follower.getPose().getY());
            telemetry.addData("Heading angle (DEGREES)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("shooter velocity (ticks/s)", outtake.getMotor().getVelocity());
            telemetry.addData("shooter power", outtake.getMotor().getPower());
            telemetry.addData("Spin Target", spinSimple.getSpinTargetPos());
            telemetry.addData("Spin Pos (deg)", spinSimple.getSpinCurrentPosition() / (144.0 / 45.0 ));
            telemetry.addData("Spin Pos (ticks)", spinSimple.getSpinCurrentPosition());
            telemetry.addData("Spin Status", spinSimple.getSpinMotor().isBusy());
            telemetry.addData("Spin Power", spinSimple.getSpinMotor().getPower());
            telemetry.addData("CURRENT_TEAM", PoseStorage.isRedAlliance ? "RED" : "BLUE");
//            Logging.LOG("SPLIT", PoseStorage.splitControls);
            telemetry.addData("HALLUCINATING", PoseStorage.shouldHallucinate);




//            Logging.LOG("LL_TX", ll.getTx());
//            Logging.LOG("LL_TY", ll.getTy());
//            if (ll.isTagDetected()) {
//                Logging.LOG("LL_DETECTED_ID", ll.getDetectedId());
//            }



            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            telemetry.update();
            telemetryM   .update();
        }
    }

    enum FinishingState {
        Intake,
        Outtake,
        None
    }

    enum Motif {
        PPG,
        PGP,
        GPP,
        None
    }
}
