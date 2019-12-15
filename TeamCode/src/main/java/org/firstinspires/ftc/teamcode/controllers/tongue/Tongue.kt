package org.firstinspires.ftc.teamcode.controllers.tongue

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.core.Controller

open class Tongue : Controller() {
    lateinit var tongueServo: Servo
    lateinit var tongueRelay: DigitalChannel

    override fun init(hardwareMap: HardwareMap) {
        tongueServo = hardwareMap.get(Servo::class.java, "tongueServo")
        tongueRelay = hardwareMap.get(DigitalChannel::class.java, "tongueRelay")
        tongueRelay.mode = DigitalChannel.Mode.OUTPUT
        tongueServo.position = 1.0
        dontLick()
    }

    fun lick(){
        tongueRelay.state = false
    }

    fun dontLick(){
        tongueRelay.state = true
    }

}