package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.controllers.PositionChassis
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoOpMode

@Autonomous(name = "AAAAAAAAAAAAAAAAAAAAAAA", group = "die")
class HALLOWIEGEHTESDIR : EctoOpMode() {
    val AAAAAAAAAAAAAAAa = PositionChassis()
    init {
        addController(AAAAAAAAAAAAAAAa)
    }
    override fun update() {
        AAAAAAAAAAAAAAAa.targetCoords = Coordinate(4.0,-2.0)
    }

}