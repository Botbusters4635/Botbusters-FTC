package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import org.firstinspires.ftc.teamcode.controllers.PositionChassis
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode
import org.firstinspires.ftc.teamcode.core.EctoOpMode
import org.firstinspires.ftc.teamcode.core.Path

@Autonomous(name = "AAAAAAAAAAAAAAAAAAAAAAA", group = "die")
class HALLOWIEGEHTESDIR : EctoLinearOpMode() {
    val AAAAAAAAAAAAAAAa = PositionChassis()

    init {
        addController(AAAAAAAAAAAAAAAa)
    }

    override fun runOpMode() {
        AAAAAAAAAAAAAAAa.runToPosition(Coordinate(1.0, 0.0))
//        AAAAAAAAAAAAAAAa.followPath(Path(
//                Coordinate(0.0,0.0),
//                Coordinate(2.0, 0.0),
//                Coordinate(0.0, 0.0)
//        ))
    }


}