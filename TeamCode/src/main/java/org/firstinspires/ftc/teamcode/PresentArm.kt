package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.controllers.arm.SynchronizedArm
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode

@Autonomous(name = "Arm presentation", group = "presentation")
class PresentArm: EctoLinearOpMode() {
    val arm = SynchronizedArm()
    init {
        controllers.add(arm)
    }

    override fun startMode() {
        arm.runToPositionBlocking(ArmPosition.HOME)
    }

    override fun runOpMode() {
        arm.runToPositionBlocking(ArmPosition.SECOND_LEVEL)
        runBlocking {
            delay(500)
        }
        arm.runToPositionBlocking(ArmPosition.HOME)
    }
}