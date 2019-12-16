package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.controllers.arm.SynchronizedArm
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode

@Autonomous(name = "AutonomousSetup", group = "other")
class MoveToSaveMode: EctoLinearOpMode(){
    val arm = SynchronizedArm()
    init {
        controllers.add(arm)
    }
    override fun runOpMode() {
        arm.runToPositionBlocking(ArmPosition.SAVE)
    }

}