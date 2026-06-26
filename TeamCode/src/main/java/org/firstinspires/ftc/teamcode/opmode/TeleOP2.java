//package org.firstinspires.ftc.teamcode.opmode;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.pedropathing.geometry.BezierPoint;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.util.Timer;
//
//import org.firstinspires.ftc.teamcode.config.command.CommandOpMode;
//import org.firstinspires.ftc.teamcode.config.util.Alliance;
//
//import static org.firstinspires.ftc.teamcode.config.Robot.defaultPose;
//
//@Configurable
//public class TeleOP2 extends CommandOpMode {
//
//    Robot r;
//    final Alliance a;
//
//    public boolean shoot = false, manual = false, field = true, hold = false, autoFlipping = false, manualFlip = false;
//    public double intakeOn = 0, dist, speed = 1;
//    public static double shootTarget = 1200;
//    private final Timer flipUpTimer = new Timer(), autoFlipTimer = new Timer();
//
//    public TeleOP2(Alliance alliance) {
//        a = alliance;
//    }
//
//    @Override
//    public void init() {
//        r = new Robot(hardwareMap, a);
//        r.f.setStartingPose(defaultPose);
//        r.t.setPowerZero();
//
//        telemetry = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();
//
//        r.g.flipDown();
//    }
//
//    @Override
//    public void init_loop() {
//        if (gamepad1.xWasPressed())
//            r.t.resetTurret();
//    }
//
//    @Override
//    public void start() {
//        r.setShootTarget();
//        r.periodic();
//        r.t.setPowerZero();
//        r.t.reset();
//        r.f.startTeleopDrive();
//
//        flipUpTimer.resetTimer();
//    }
//
//    @Override
//    public void loop() {
//        r.periodic();
//
//        if (!hold)
//            if (field)
//                r.f.setTeleOpDrive(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x, speed * -gamepad1.right_stick_x, false, r.a == Alliance.BLUE ? Math.toRadians(180) : 0);
//            else
//                r.f.setTeleOpDrive(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x, speed * -gamepad1.right_stick_x, true);
//
//        if (flipUpTimer.getElapsedTimeSeconds() > 1 && !r.g.isDown() && manualFlip)
//            gamepad1.rumbleBlips(1);
//
//        if (gamepad1.rightBumperWasPressed())
//            if (intakeOn == 1)
//                intakeOn = 0;
//            else
//                intakeOn = 1;
//
//        if (gamepad1.dpadDownWasPressed())
//            if (intakeOn == 2)
//                intakeOn = 0;
//            else
//                intakeOn = 2;
//
//        if (intakeOn == 1)
//            r.i.spinIn();
//        else if (intakeOn == 2)
//            r.i.spinOut();
//        else
//            r.i.spinOff();
//
//        if (shoot) {
//            r.s.on();
//            r.t.on();
//
//            if (manual) {
//                r.t.manual(-gamepad1.right_trigger + gamepad1.left_trigger);
//                r.s.setTarget(shootTarget);
//            } else {
////                dist = r.getShootTarget().distanceFrom(r.f.getPose());
//                boolean close = r.f.getPose().getY() > 48;
////                r.s.forDistance(dist, close);
//                r.s.forPose(r.f.getPose(), r.getShootTarget(), close);
//                r.t.face(r.getShootTarget(), r.f.getPose());
//                r.t.automatic();
//            }
//        } else {
//            r.s.off();
//            r.t.off();
//        }
//
//        if (gamepad1.aWasPressed())
//            if (manualFlip) {
//                flipUpTimer.resetTimer();
//                r.g.toggle();
//                autoFlipping = false;
//            } else {
//                autoFlipping = true;
//                autoFlipTimer.resetTimer();
//            }
//
//        if (!manualFlip && autoFlipping && shoot) {
//            if (autoFlipTimer.getElapsedTimeSeconds() > 1.9) {
//                r.g.flipDown();
//                autoFlipping = false;
//            } else if (autoFlipTimer.getElapsedTimeSeconds() > 1.6)
//                r.g.flipUp();
//            else if (autoFlipTimer.getElapsedTimeSeconds() > 1.25)
//                r.g.flipDown();
//            else if (autoFlipTimer.getElapsedTimeSeconds() > 1)
//                r.g.flipUp();
//            else if (autoFlipTimer.getElapsedTimeSeconds() > .7)
//                r.g.flipDown();
//            else if (autoFlipTimer.getElapsedTimeSeconds() > .55)
//                r.g.flipUp();
//            else if (autoFlipTimer.getElapsedTimeSeconds() > 0.25)
//                r.g.flipDown();
//            else if (autoFlipTimer.getElapsedTimeSeconds() > 0)
//                r.g.flipUp();
//        } else {
//            if (!r.g.isDown() && !manualFlip)
//                r.g.flipDown();
//        }
//
//
//        if (gamepad1.leftStickButtonWasPressed())
//            manualFlip = !manualFlip;
//
//        if (gamepad1.bWasPressed())
//            shoot = !shoot;
//
//        if (gamepad1.dpadUpWasPressed()) {
//            if (r.a.equals(Alliance.BLUE)) {
//                r.f.setPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
//            } else {
//                r.f.setPose(new Pose(8, 6.25, Math.toRadians(0)));
//            }
//        }
//
//        if (gamepad1.dpadLeftWasPressed())
//            manual = !manual;
//
//        if (gamepad1.dpadRightWasPressed())
//            field = !field;
//
//        if (gamepad1.yWasPressed()) {
//            hold = !hold;
//
//            if (hold) {
//                r.f.holdPoint(new BezierPoint(r.f.getPose()), r.f.getHeading(), false);
//            } else {
//                r.f.startTeleopDrive();
//            }
//        }
//
//        if (gamepad1.xWasPressed())
//            r.t.resetTurret();
//
//        if (gamepad1.left_bumper)
//            speed = 0.5;
//        else
//            speed = 1.0;
//
//        telemetry.addData("LoopTime Hz", r.getLoopTimeHz());
//        telemetry.addData("Abs X", Math.abs(r.getShootTarget().getX()-r.f.getPose().getX()));
//        telemetry.addData("Abs Y", Math.abs(r.getShootTarget().getY()-r.f.getPose().getY()));
//        telemetry.addData("Shoot Target", shootTarget);
//        telemetry.addLine();
//        telemetry.addData("Follower Pose", r.f.getPose().toString());
//        telemetry.addData("Shooter Velocity", r.s.getVelocity());
//        telemetry.addData("Shooter Target", r.s.getTarget());
//        telemetry.addData("Shooter Distance", dist);
//        telemetry.addData("Turret Yaw", r.t.getYaw());
//        telemetry.addData("Turret Target", r.t.getTurretTarget());
//        telemetry.addData("Turret Ticks", r.t.getTurret());
//        telemetry.addData("Shooter On", shoot);
//        telemetry.addData("Gate Closed", r.g.isDown());
//        telemetry.addData("Automatic Flipping Running", autoFlipping);
//        telemetry.addData("Automatic Flipping Timer", autoFlipTimer.getElapsedTimeSeconds());
//        telemetry.addLine("Manual Flipper: " + manualFlip);
//        telemetry.addData("Distance from Target", dist);
//        telemetry.addData("Manual Shooter + Turret", manual);
//        telemetry.addData("Field Centric", field);
//        telemetry.addData("Hold Position", hold);
//        telemetry.update();
//    }
//
//
//    @Override
//    public void stop() {
//        r.saveEnd();
//    }
//}