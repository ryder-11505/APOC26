package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.CommandBuilder
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import com.pedropathing.ivy.commands.Commands;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit


class shooter(hardwareMap: HardwareMap) {
    companion object PARAMS {
        @JvmField
        var speedSuperShort = ((21.5 * 10.0) / (2.0 * Math.PI)) * 28.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/628 = power

        @JvmField
        var speedShort = ((22.0 * 10.0) / (2.0 * Math.PI)) * 28.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/62

        @JvmField
        var speedLong = 990.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/628 = power

        @JvmField
        var speed = 1300.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/628 = power

        @JvmField
        var open = 0.5125

        @JvmField
        var closed = 0.25


    }

    val motor = hardwareMap.get(DcMotorEx::class.java, "outL")
    val motor2 = hardwareMap.get(DcMotorEx::class.java, "outR")
    val servo = hardwareMap.get(Servo::class.java, "servo")

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.setVelocityPIDFCoefficients(34.0,0.0,0.0,20.0)
        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motor2.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor2.setVelocityPIDFCoefficients(34.0,0.0,0.0,20.0)
//        motor.setVelocityPIDFCoefficients(1.17025,0.117025,0.0,11.7025)
        servo.position = closed

    }


    fun open() {
        servo.position = open
    }

    fun close2() {
        servo.position = closed
    }

    fun close() {
        servo.position = closed
    }


    fun setPower(RS: Double) {
        motor.velocity = RS
        motor2.velocity = -RS
//        if ((motor.velocity) < (RS)) {
//            servo.position = closed
//        } else {
//            servo.position = open
//        }
    }

    fun stopShoot() {
        motor.velocity = 0.0
        motor2.velocity = 0.0
        servo.position = closed
    }

}
