package org.firstinspires.ftc.teamcode.subsystems

import androidx.xr.runtime.Config
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.max


class turret(hardwareMap: HardwareMap) {
    companion object PARAMS {

        @JvmField
        var ticksPerDegree: Double = 3.5

        private const val encoderCPR = 28.0
        private var gearRatio = (ticksPerDegree * 360.0) / encoderCPR // ≈ 41.2380952381

        @JvmField
        var P = 25.0

        @JvmField
        var I = 0.0

        @JvmField
        var D = 7.0

        @JvmField
        var F = 0.0
    }

    val spinMotor = hardwareMap.get(DcMotorEx::class.java, "spin")
    val servo = hardwareMap.get(Servo::class.java, "servo2")

    val spinCurrentPosition get() = spinMotor.currentPosition
    val spinTargetPos get() = spinMotor.targetPosition

    private fun initMotor() {
        spinMotor.targetPosition = 0
        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        spinMotor.power = 0.1
        spinMotor.targetPositionTolerance = 2
    }

    init {
        initMotor()
        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        spinMotor.setPositionPIDFCoefficients(P)
        spinMotor.setVelocityPIDFCoefficients(P, I, D, F)
        spinMotor.direction = DcMotorSimple.Direction.FORWARD
        servo.position = 0.50
    }

    fun track(degrees: Double, offset: Double) {
        val degreesWithOffset = degrees + offset
        var safeDegrees = degreesWithOffset
        if (degreesWithOffset > 180) {
            safeDegrees = degreesWithOffset - 360
        } else if (degreesWithOffset < -180) {
            safeDegrees = 360 - degreesWithOffset
        } else if (IntRange(-180, 180).contains(degrees.toInt())) {
            safeDegrees = degreesWithOffset
        }
        val targetPosition = ((safeDegrees * ticksPerDegree).toInt())
        spinMotor.targetPosition = targetPosition
        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        spinMotor.power = 1.0
        if (abs(((safeDegrees * ticksPerDegree).toInt()) - ((safeDegrees * ticksPerDegree).toInt())) < spinMotor.targetPositionTolerance && abs(((safeDegrees * ticksPerDegree).toInt()) - spinCurrentPosition) < spinMotor.targetPositionTolerance) {
            spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            spinMotor.power = 0.0
        }
        if (IntRange(-1, 1).contains(((safeDegrees * ticksPerDegree).toInt()) - spinCurrentPosition)){
            spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            spinMotor.power = 0.0
        }
    }

    fun hoodAngle(angle: Double) {
        val position = 0.0333333 * angle - 0.333333
        servo.position = position
    }

    fun resetEncoder() {
        spinMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }


}
