package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
@TeleOp(name = "Pinpoint Diagnostic", group = "Test")
public class PinpointDiagnostic extends LinearOpMode {

    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        odo.setOffsets(4.9, 4.9, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        odo.resetPosAndIMU();
        sleep(300);

        telemetry.addLine("Keep robot still - calibrating...");
        telemetry.update();
        sleep(1000);

        telemetry.addLine("Ready! Push robot and watch values.");
        telemetry.addLine("  FORWARD push  -> Y should increase");
        telemetry.addLine("  RIGHT strafe  -> X should increase");
        telemetry.addLine("  CCW rotation  -> Heading should increase");
        telemetry.addLine("");
        telemetry.addLine("Press START when ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double x       = pos.getX(DistanceUnit.INCH);
            double y       = pos.getY(DistanceUnit.INCH);
            double heading = pos.getHeading(AngleUnit.DEGREES);

            telemetry.addLine("---- Push robot by hand ----");
            telemetry.addData("X (strafe right = +)", "%.2f in", x);
            telemetry.addData("Y (forward      = +)", "%.2f in", y);
            telemetry.addData("Heading (CCW    = +)", "%.2f deg", heading);
            telemetry.addLine("");
            telemetry.addLine("Expected signs:");
            telemetry.addData("Push FORWARD",  "Y increases (+)");
            telemetry.addData("Strafe RIGHT",  "X increases (+)");
            telemetry.addData("Rotate CCW",    "Heading increases (+)");
            telemetry.addLine("");
            telemetry.addData("X pod raw", odo.getEncoderX());
            telemetry.addData("Y pod raw", odo.getEncoderY());
            telemetry.update();
        }
    }
}