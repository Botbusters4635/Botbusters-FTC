package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.controllers.PositionChassis
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode
import org.firstinspires.ftc.teamcode.core.Path

@Autonomous(name = "TrayOnly", group = "ezpz")
class TrayOnlyAutonomous : EctoLinearOpMode() {
    val chassis = PositionChassis()

    init {
        addController(chassis)
    }


    override fun runOpMode() {
        chassis.followPath(Path(
                Coordinate(0.0, 0.0),
                Coordinate(0.5, 0.0),
                Coordinate(1.25, -0.75)
        ))
        chassis.turnToAngle(0.0)
        chassis.runToPosition(Coordinate(0.5, -0.75), true)
        chassis.runToPosition(Coordinate(1.5, -0.75))

    }


}