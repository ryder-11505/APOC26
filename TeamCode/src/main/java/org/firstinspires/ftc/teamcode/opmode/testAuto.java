package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.commands.Commands.*;

import org.firstinspires.ftc.teamcode.config.pedro.Constants;

@Autonomous(name = "9 ball + gate", group = "Examples")
public class testAuto extends LinearOpMode {

    private Follower follower;
    private Paths paths;

    @Override
    public void runOpMode() {

        // Reset the scheduler so no leftover commands from a previous run linger
        Scheduler.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(24.150, 127.103, Math.toRadians(143)));

        paths = new Paths(follower);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Schedule the entire auto as one command
        schedule(autoRoutine());

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    private Command autoRoutine() {
        return sequential(
                /* Move to first shoot location */
                follow(follower, paths.Movetoshoot1),
                /* TODO: replace with real shooter command */
                instant(() -> { /* shooter.shoot(); */ }),
                waitMs(500),

                /* Move to intake 1, then intake */
                follow(follower, paths.Movetointake1, true),
                follow(follower, paths.Intake1, true),
                instant(() -> { /* intake.start(); */ }),
                waitMs(300),
                instant(() -> { /* intake.stop(); */ }),

                /* Back to shoot */
                follow(follower, paths.Movetoshoot2, true),
                instant(() -> { /* shooter.shoot(); */ }),
                waitMs(500),

                /* Move to gate, hit it */
                follow(follower, paths.Movetogate1, true),
                follow(follower, paths.Hitgate1, true),

                /* Move to intake 2, then intake */
                follow(follower, paths.Movetointake2, true),
                follow(follower, paths.Intake2, true),
                instant(() -> { /* intake.start(); */ }),
                waitMs(300),
                instant(() -> { /* intake.stop(); */ }),

                /* Back to shoot */
                follow(follower, paths.Movetoshoot3, true),
                instant(() -> { /* shooter.shoot(); */ }),
                waitMs(500),

                /* Drive out */
                follow(follower, paths.Driveout, true)
        );
    }

    // ---- Paths class ----
    public static class Paths {

        public PathChain Movetoshoot1;
        public PathChain Movetointake1;
        public PathChain Intake1;
        public PathChain Movetoshoot2;
        public PathChain Movetogate1;
        public PathChain Hitgate1;
        public PathChain Movetointake2;
        public PathChain Intake2;
        public PathChain Movetoshoot3;
        public PathChain Driveout;

        public Paths(Follower follower) {

            Movetoshoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(24.150, 127.103),
                                    new Pose(65.944, 102.953)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            Movetointake1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(65.944, 102.953),
                                    new Pose(53.713, 93.368),
                                    new Pose(45.477, 83.785)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Intake1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(45.477, 83.785),
                                    new Pose(13.907, 83.421)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Movetoshoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(13.907, 83.421),
                                    new Pose(44.009, 89.019),
                                    new Pose(66.112, 102.617)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Movetogate1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(66.112, 102.617),
                                    new Pose(52.981, 80.318),
                                    new Pose(30.505, 73.570)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Hitgate1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(30.505, 73.570),
                                    new Pose(16.692, 73.215)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Movetointake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(16.692, 73.215),
                                    new Pose(45.981, 76.710),
                                    new Pose(57.252, 68.888),
                                    new Pose(41.103, 59.402)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(41.103, 59.402),
                                    new Pose(17.252, 59.355)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Movetoshoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(17.252, 59.355),
                                    new Pose(43.879, 77.827),
                                    new Pose(65.963, 103.056)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Driveout = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(65.963, 103.056),
                                    new Pose(58.224, 85.664),
                                    new Pose(55.252, 66.514)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}