package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.abs

import com.bylazar.configurables.annotations.Configurable


class turret(hardwareMap: HardwareMap) {

    @Configurable
    companion object PARAMS {

        @JvmField var ticksPerDegree: Double = 8.79083 * 49/45

        @JvmField var P = 12.0
        @JvmField var I = 2.0
        @JvmField var D = 3.0
        @JvmField var F = 0.0

        // Trim added to every turret angle command — positive = clockwise
        @JvmField var turretTrim: Double = 2.0

        // Tuned table: (distanceMM, servoPos, shooterRPM)
//        val HOOD_TABLE = listOf(
//            Triple(500.0,  1.0000, 1000.0),
//            Triple(1000.0, 0.9400, 1020.0),
//            Triple(1500.0, 0.8700, 1200.0),
//            Triple(2000.0, 0.8000, 1300.0),
//            Triple(2500.0, 0.7700, 1330.0),
//            Triple(3000.0, 0.7700, 1430.0),
//        )

        val HOOD_TABLE = listOf(
            Triple(500.0,  1.0000, 940.0),
            Triple(1000.0, 0.9400, 980.0),
            Triple(1500.0, 0.8700, 1140.0),
            Triple(2000.0, 0.8000, 1240.0),
            Triple(2500.0, 0.7700, 1290.0),
            Triple(3000.0, 0.7700, 1390.0),
        )
    }

    // Hardware
    val spinMotor         = hardwareMap.get(DcMotorEx::class.java, "spin")
    val servo             = hardwareMap.get(Servo::class.java, "servo2")


    val spinCurrentPosition get() = spinMotor.currentPosition
    val spinTargetPos       get() = spinMotor.targetPosition

    init {
        spinMotor.targetPosition = 0
        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        spinMotor.power = 0.1
        spinMotor.targetPositionTolerance = 2
        spinMotor.setPositionPIDFCoefficients(P)
        spinMotor.setVelocityPIDFCoefficients(P, I, D, F)
        spinMotor.direction = DcMotorSimple.Direction.REVERSE
        servo.position = 1.0
    }

    fun track(degrees: Double, offset: Double) {
        val degreesWithOffset = degrees + offset + turretTrim
        val safeDegrees = when {
            degreesWithOffset >  180 -> degreesWithOffset - 360
            degreesWithOffset < -180 -> degreesWithOffset + 360
            else                     -> degreesWithOffset
        }
        val targetPosition = (safeDegrees * ticksPerDegree).toInt()
        spinMotor.targetPosition = targetPosition
        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        spinMotor.power = 1.0

        if (abs(targetPosition - spinCurrentPosition) < spinMotor.targetPositionTolerance) {
            spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            spinMotor.power = 0.0
        }
    }

    fun servoPos(pos: Double) {
        servo.position = pos
    }

    fun hoodFromDistance(distanceMM: Double): Double {
        val table = HOOD_TABLE
        if (distanceMM <= table.first().first) { servo.position = table.first().second; return table.first().second }
        if (distanceMM >= table.last().first)  { servo.position = table.last().second;  return table.last().second  }
        val lower = table.last  { it.first <= distanceMM }
        val upper = table.first { it.first >  distanceMM }
        val t = (distanceMM - lower.first) / (upper.first - lower.first)
        val pos = lower.second + t * (upper.second - lower.second)
        servo.position = pos
        return pos
    }

    fun rpmFromDistance(distanceMM: Double): Double {
        val table = HOOD_TABLE
        val baseRPM = when {
            distanceMM <= table.first().first -> table.first().third
            distanceMM >= table.last().first  -> table.last().third
            else -> {
                val lower = table.last  { it.first <= distanceMM }
                val upper = table.first { it.first >  distanceMM }
                val t = (distanceMM - lower.first) / (upper.first - lower.first)
                lower.third + t * (upper.third - lower.third)
            }
        }
        return baseRPM
    }

    /**
     * Sets hood servo and returns voltage-compensated RPM for the shooter.
     * Call every loop when shooting:
     *   val rpm = turret.aimForDistance(distanceMM)
     *   shooter.setPower(rpm, intakeRunning)
     */
    fun aimForDistance(distanceMM: Double): Double {
        hoodFromDistance(distanceMM)
        return rpmFromDistance(distanceMM)
    }

    fun resetEncoder() {
        spinMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        spinMotor.targetPosition = 0
        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        spinMotor.power = 0.1
    }
}