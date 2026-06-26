package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit


class intake(hardwareMap: HardwareMap) {
    companion object PARAMS {
        @JvmField // Forward intake power
        var P_Intake: Double = 15.0

        @JvmField // Speed while intaking, if it is flying past lower this number
        var speed = 0.85
    }

    val motor = hardwareMap.get(DcMotorEx::class.java, "in")
//    val motor2 = hardwareMap.get(DcMotorEx::class.java, "in2")

    // ball1 is the first ball to be intaked and closest to the shooter, ball2 is in the middle,
    // and ball3 is the most recently intaked and closest to the intake

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.setCurrentAlert(4.5, CurrentUnit.AMPS)
//        motor2.setCurrentAlert(4.5, CurrentUnit.AMPS)
    }


    fun intake() {
        motor.power = speed
//        motor2.power = speed
    }

    fun outake() {
        motor.power = - speed
//        motor2.power = - speed
    }

    fun stopIntake() {
        motor.power = 0.0
//        motor2.power = 0.0
    }

}
