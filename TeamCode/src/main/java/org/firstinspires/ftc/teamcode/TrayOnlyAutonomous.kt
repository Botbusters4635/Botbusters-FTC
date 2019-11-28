package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
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
                Coordinate(0.3, 0.0)
        ))

        /**
         * Asume que lo de los servos estará por atras
         */
        chassis.turnToAngle(180.0)
        chassis.runToPosition(Coordinate(0.5, 0.0))
        /***
         * Enable grabbing cosa
         */


        /**
         * Mover charola más cerca a la zona
         */

        chassis.runToPosition(Coordinate(0.2, 0.0))

        /**
         * Girar para acomodarla y empujarla contra la zona
         */
        chassis.turnToAngle(-90.0)
        chassis.runToPosition(Coordinate(chassis.getCurrentCords().x, chassis.getCurrentCords().y + 0.25))

        /**
         * Ir a estacionarse
         */
        chassis.runToPosition(Coordinate(chassis.getCurrentCords().x, -0.7))

    }


}