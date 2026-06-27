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

import com.pedropathing.control.PIDFController
import com.pedropathing.control.PIDFCoefficients


class shooter(hardwareMap: HardwareMap) {
    companion object PARAMS {
        @JvmField
        var speedSuperShort =
            ((21.5 * 10.0) / (2.0 * Math.PI)) * 28.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/628 = power

        @JvmField
        var speedShort =
            ((22.0 * 10.0) / (2.0 * Math.PI)) * 28.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/62

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

    // --- MANUALLY CALIBRATED COEFFICIENTS FOR 1.2:1 HIGH MASS WHEEL ---
    // P = 0.00015 -> A very small adjustment weight to gently push towards target speed
    // I = 0.0     -> Kept at zero to completely prevent runaway Integral Windup
    // D = 0.0     -> Kept at zero for speed stability
    // F = 0.00018 -> The absolute base throttle line. 500 TPS * 0.00018 = 0.09 (9% base voltage)
    private val coefficients = PIDFCoefficients(0.0007, 0.0, 0.0, 0.000295)
    private val controller = PIDFController(coefficients)

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        // Unlocks absolute top speed potential by bypassing the 80% REV Hub speed limits
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        servo.position = closed
    }

    fun open() { servo.position = open }
    fun close() { servo.position = closed }
    fun close2() { servo.position = closed }

    /**
     * Set the shooter velocity using target RPM driven by the Pedro Pathing PIDF Engine
     */
    fun setPower(RS: Double) {
        if (RS <= 0.1) {
            stopShoot()
            return
        }

        // 1. Convert incoming target RPM directly to Target Ticks Per Second
        val targetTPS = (RS / 60.0) * 28.0

        // 2. Pass target parameters to the Pedro Pathing lifecycle methods
        controller.setTargetPosition(targetTPS)
        controller.updateFeedForwardInput(targetTPS)

        // 3. Read the verified current hardware speed from the encoder
        val currentTPS = motor.velocity

        // 4. Update the timeline tracking loops and execute the calculation
        controller.updatePosition(currentTPS)
        val calculatedPower = controller.run()

        // 5. Coerce output power securely between 0.0 and 1.0 (prevents running backwards)
        val finalPower = calculatedPower.coerceIn(0.0, 1.0)

        motor.power = finalPower
        motor2.power = -finalPower
    }

    fun stopShoot() {
        motor.power = 0.0
        motor2.power = 0.0
        controller.reset() // Safely clears historical time deltas and error memories
        servo.position = closed
    }
}

//    init {
//        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
//        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
//        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
//        motor2.mode = DcMotor.RunMode.RUN_USING_ENCODER
////        motor.setVelocityPIDFCoefficients(1.17025,0.117025,0.0,11.7025)
//
//        motor.setVelocityPIDFCoefficients(4.0, 1.0, 0.25, 7.0)
//        motor2.setVelocityPIDFCoefficients(4.0, 1.0, 0.25, 7.0)
//
//        servo.position = closed
//
//    }
//
//
//    fun open() {
//        servo.position = open
//    }
//
//    fun close2() {
//        servo.position = closed
//    }
//
//    fun close() {
//        servo.position = closed
//    }
//
//
//    fun setPower(targetRPM: Double) {
//        // 1. Define the encoder ticks for your internal motor shaft (REV HD Hex = 28 ticks/rev)
//        val encoderTicksPerRev = 28.0
//
//        // 2. Convert RPM into Ticks Per Second (TPS)
//        val targetTPS = (targetRPM / 60.0) * encoderTicksPerRev
//
//        // 3. Command the motor using the correct TPS units
//        motor.velocity = targetTPS
//        motor2.velocity = -targetTPS
//    }
//
//    fun stopShoot() {
//        motor.velocity = 0.0
//        motor2.velocity = 0.0
//        servo.position = closed
//    }
//}