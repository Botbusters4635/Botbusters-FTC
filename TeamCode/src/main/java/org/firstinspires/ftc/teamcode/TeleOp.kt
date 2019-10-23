package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.EctoTeleOp

@TeleOp(name = "TeleOp")
class TeleOp: EctoTeleOp() {
    init {
        addController()
    }
}