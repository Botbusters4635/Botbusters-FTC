package org.firstinspires.ftc.teamcode.controllers.clamp

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.core.Controller

class Clamp : Controller() {

    private lateinit var clampServo: Servo
    private lateinit var turningServo: Servo

    var angle: Double = 0.0
        set(value) {
            turningServo.position = (180 - value) / 180
            field = value
        }

    var power: Double = 0.0
        set(value) {
            val position = 1.0 - value * 0.2
            clampServo.position = position
            field = value
        }


    override fun init(hardwareMap: HardwareMap) {
        clampServo = hardwareMap.get(Servo::class.java, "clampServo")
        turningServo = hardwareMap.get(Servo::class.java, "turningServo")
        clampServo.direction = Servo.Direction.REVERSE
        turningServo.direction = Servo.Direction.REVERSE
    }

    override fun update(timeStep: Double) {
    }
}