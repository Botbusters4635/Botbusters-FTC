package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.systems.Chassis

@TeleOp(name = "TeleOp")
class TeleOp: OpMode() {
    val chassis = Chassis()

    override fun init() {
        hardwareMap
    }

    override fun loop() {

    }

}