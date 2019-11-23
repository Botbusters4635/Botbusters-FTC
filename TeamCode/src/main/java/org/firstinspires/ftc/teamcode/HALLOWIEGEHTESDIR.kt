package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.CoroutineExceptionHandler
import org.firstinspires.ftc.teamcode.Controllers.PositionChassis
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoOpMode

@Autonomous(name = "AAAAAAAAAAAAAAAAAAAAAAA", group = "die")
class HALLOWIEGEHTESDIR : EctoOpMode() {
    val AAAAAAAAAAAAAAAa = PositionChassis()
    init {
        addController(AAAAAAAAAAAAAAAa)
    }
    override fun update() {
        AAAAAAAAAAAAAAAa.targetCoords = Coordinate(2.0,0.0)
    }

}