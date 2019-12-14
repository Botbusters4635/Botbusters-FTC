package org.firstinspires.ftc.teamcode.controllers.arm

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.controllers.clamp.Clamp

class SynchronizedArm : PositionArm() {
    private var currentState = ArmState.GO_TARGET

    private var clawTurnStartTime = SystemClock.elapsedRealtime() / 1000.0
    private val clawTurnTime = 0.8 //Seconds needed for claw to turn to position
    private var clawTurning = false

    var currentTargetCoord = ArmPosition.HOGAR.coordinate

    val clamp = Clamp()


    override fun init(hardwareMap: HardwareMap) {
        super.init(hardwareMap)
        controllers.add(clamp)
    }

    override fun start() {
        super.start()
        clamp.angle = TURN_POS.CLOSED.value
    }

    override fun update(timeStep: Double) {
        super.update(timeStep)

        telemetry.addData("currentState", currentState)
        telemetry.addData("targetCoord", targetCoordinate)

        when (currentState) {
            ArmState.EXCHANGE_FRONT_TO_BACK -> {
                targetCoordinate = ArmPosition.EXCHANGE.coordinate
                if (currentCoordinate.closeTo(ArmPosition.EXCHANGE.coordinate)) {
                    clamp.angle = 180.0
                    if (!clawTurning) {
                        clawTurning = true
                        clawTurnStartTime = SystemClock.elapsedRealtime() / 1000.0
                    } else if (SystemClock.elapsedRealtime() / 1000.0 - clawTurnStartTime > clawTurnTime) {
                        clawTurning = false
                        clamp.angle = 180.0
                        currentState = ArmState.GO_TARGET
                    }
                }



            }
            ArmState.EXCHANGE_BACK_TO_FRONT -> {
                targetCoordinate = ArmPosition.EXCHANGE.coordinate
                if (currentCoordinate.closeTo(ArmPosition.EXCHANGE.coordinate)) {
                    clamp.angle = 0.0
                    if (!clawTurning) {
                        clawTurning = true
                        clawTurnStartTime = SystemClock.elapsedRealtime() / 1000.0
                    } else if (SystemClock.elapsedRealtime() / 1000.0 - clawTurnStartTime > clawTurnTime) {
                        clawTurning = false
                        clamp.angle = 0.0
                        currentState = ArmState.GO_TARGET
                    }
                }



            }
            ArmState.GO_TARGET -> {
                targetCoordinate = currentTargetCoord
            }
        }
    }

    override fun moveToPosition(position: ArmPosition) {
        if (position.coordinate != targetCoordinate && !clawTurning) {
            currentTargetCoord = position.coordinate
            val targetAngles = kinematics.calculateInverseKinematics(currentTargetCoord)
            if (currentAngles.lowerAngle + currentAngles.upperAngle < 10.0 &&  targetAngles.lowerAngle + targetAngles.upperAngle > 10.0) {
                currentState = ArmState.EXCHANGE_FRONT_TO_BACK
            } else if(currentAngles.lowerAngle + currentAngles.upperAngle > 10.0 &&  targetAngles.lowerAngle + targetAngles.upperAngle < 10.0) {
                currentState = ArmState.EXCHANGE_BACK_TO_FRONT
            }
        }
    }
}