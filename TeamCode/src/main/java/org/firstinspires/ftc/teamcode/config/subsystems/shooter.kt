package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

import com.pedropathing.control.PIDFController
import com.pedropathing.control.PIDFCoefficients


class shooter(hardwareMap: HardwareMap) {
    companion object PARAMS {
        @JvmField var speedSuperShort = ((21.5 * 10.0) / (2.0 * Math.PI)) * 28.0
        @JvmField var speedShort      = ((22.0 * 10.0) / (2.0 * Math.PI)) * 28.0
        @JvmField var speedLong       = 990.0
        @JvmField var speed           = 1300.0

        @JvmField var open   = 0.25
        @JvmField var closed = 0.5125

        // Extra RPM added after the first ball clears
        @JvmField var boostRPM: Double = 150.0

        // Delay before boost kicks in — first ball clears in ~500ms
        @JvmField var boostDelayMs: Long = 500
    }

    val motor  = hardwareMap.get(DcMotorEx::class.java, "outL")
    val motor2 = hardwareMap.get(DcMotorEx::class.java, "outR")
    val servo  = hardwareMap.get(Servo::class.java, "servo")

    private val coefficients = PIDFCoefficients(0.0007, 0.0, 0.0, 0.000295)
    private val controller   = PIDFController(coefficients)

    private var isOpen = false

    // Timestamp when intake started, -1 means not running
    private var intakeStartMs: Long = -1L

    init {
        motor.zeroPowerBehavior  = DcMotor.ZeroPowerBehavior.FLOAT
        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motor.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        servo.position = closed
    }

    fun open()   { servo.position = open   }
    fun close()  { servo.position = closed }
    fun close2() { servo.position = closed }

    fun toggleGate() {
        isOpen = !isOpen
        servo.position = if (isOpen) open else closed
    }

    /**
     * boost = true when intake is running.
     * Boost only activates after boostDelayMs so the first ball is unaffected.
     * intakeRunning should be set to true when intake starts, false when it stops.
     */
    fun setPower(RS: Double, intakeRunning: Boolean = false) {
        if (RS <= 0.1) {
            stopShoot()
            return
        }

        // Track when intake started
        if (intakeRunning && intakeStartMs < 0) {
            intakeStartMs = System.currentTimeMillis()
        } else if (!intakeRunning) {
            intakeStartMs = -1L
        }

        // Only boost after the first ball has cleared
        val boostActive = intakeRunning
                && intakeStartMs >= 0
                && (System.currentTimeMillis() - intakeStartMs) >= boostDelayMs

        val activeRPM = if (boostActive) RS + boostRPM else RS

        val targetTPS = (activeRPM / 60.0) * 28.0
        controller.setTargetPosition(targetTPS)
        controller.updateFeedForwardInput(targetTPS)
        val currentTPS = motor.velocity
        controller.updatePosition(currentTPS)
        val calculatedPower = controller.run()
        val finalPower = calculatedPower.coerceIn(0.0, 1.0)
        motor.power  =  finalPower
        motor2.power = -finalPower
    }

    fun stopShoot() {
        motor.power  = 0.0
        motor2.power = 0.0
        intakeStartMs = -1L
        controller.reset()
        servo.position = closed
    }
}