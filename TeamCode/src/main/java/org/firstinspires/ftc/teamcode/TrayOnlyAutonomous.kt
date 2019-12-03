package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.PositionChassis
import org.firstinspires.ftc.teamcode.controllers.TrayHolder
import org.firstinspires.ftc.teamcode.controllers.TrayHolderPosition
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.controllers.arm.SynchronizedArm
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode

@Autonomous(name = "TrayOnly", group = "Red")
class TrayOnlyAutonomous : EctoLinearOpMode() {
    val chassis = PositionChassis()
    val arm = SynchronizedArm()

    init {
        addController(chassis)
        addController(arm)
    }

    override fun startMode() {
        chassis.heading = 180.0
        chassis.turnToAngleBlocking(180.0)
        arm.moveToPosition(ArmPosition.INTAKE)
    }

    override fun runOpMode() {

        chassis.runToPositionBlocking(Coordinate(0.15, 0.0))

        chassis.runToPositionBlocking(Coordinate(0.15, 0.22))

        chassis.runToPositionBlocking(Coordinate(0.87, 0.22))

        chassis.turnToAngleBlocking(-145.0)

        chassis.runToPositionBlocking(Coordinate(0.6, 0.15))

        chassis.turnToAngleBlocking(180.0)

        chassis.turnToAngleBlocking(-90.0)

        chassis.runToPositionBlocking(Coordinate(0.7, -0.075))

        chassis.turnToAngleBlocking(-90.0)

        chassis.runToPositionBlocking(Coordinate(0.65, -0.075))

        chassis.turnToAngleBlocking(0.0)

        chassis.runToPositionBlocking(Coordinate(0.2, -0.075))

        runBlocking {
            delay(5000)
        }

//        chassis.runToPositionBlocking(Coordinate(0.75, 0.0))

//        chassis.runToPositionBlocking(Coordinate(0.1, -0.5))
        /***
         * Enable grabbing cosa
         */
//        trayHolder.setPosition(TrayHolderPosition.Grab)
//        runBlocking {
//            delay(250)
//        }
//
//        /**q
//
//        chassis.runToPositionBlocking(Coordinate(0.5, 0.0))
//
//
//        /**
//         * Girar para acomodarla y empujarla contra la zona
//         */
//        chassis.turnToAngleBlocking(90.0)
//        arm.moveToPosition(ArmPosition.PASSBRIDGE)
//        chassis.runToPositionBlocking(Coordinate(chassis.getCurrentCords().x, chassis.getCurrentCords().y - 0.25))
//
//        trayHolder.setPosition(TrayHolderPosition.Release)
//        runBlocking {
//            delay(250)
//        }
//
//        /**
//         * Ir a estacionarse
//         */
//        chassis.runToPositionBlocking(Coordinate(chassis.getCurrentCords().x, 1.0))
    }


}