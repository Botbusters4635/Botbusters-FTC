package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.*
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.controllers.arm.SynchronizedArm
import org.firstinspires.ftc.teamcode.controllers.clamp.Clamp
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode

@Autonomous(name = "BlueAuto", group = "Blue")
class BlueAuto : EctoLinearOpMode() {
    val chassis = PositionChassis()
    val arm = SynchronizedArm()
    val intake = Intake()
    val clamp = Clamp()

    init {
        addController(chassis)
        addController(arm)
        addController(intake)
        addController(clamp)
    }

    override fun startMode() {
        chassis.heading = 180.0
        chassis.turnToAngleBlocking(180.0)
    }

    override fun runOpMode() {

        chassis.maxAutoVx = 0.3


        chassis.runToPositionBlocking(Coordinate(0.36, 0.0))

        arm.moveToPosition(ArmPosition.INTAKE)

//        chassis.maxAutoVx = 0.1
        chassis.runToPositionBlocking(Coordinate(0.1, 0.0))

//        chassis.turnToAngleBlocking(20.0)

        chassis.turnToAngleBlocking(-90.0)

        chassis.maxAutoVx = 0.3

        arm.runToPositionBlocking(ArmPosition.PASSBRIDGE)

        chassis.runToPositionBlocking(Coordinate(0.1, -0.46))

        chassis.turnToAngleBlocking(-50.0)

        intake.power = -1.0

        chassis.moveTimed(MecanumMoveCommand(vx = 0.25, theta = chassis.heading), 3.25)

        chassis.runToPositionBlocking(Coordinate(0.2, -0.66))

        chassis.turnToAngleBlocking(-90.0)

        intake.power = 0.0

        //leAFFAE

        chassis.runToPositionBlocking(Coordinate(0.2, -0.1))

        arm.runToPositionBlocking(ArmPosition.HOME)

        clamp.power = 1.0

        runBlocking {
            delay(500)
        }

        arm.runToPositionBlocking(ArmPosition.SECOND_LEVEL)

        clamp.power = 0.0

        arm.runToPositionBlocking(ArmPosition.PASSBRIDGE)

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


