package org.firstinspires.ftc.teamcode.controllers.arm

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.controllers.clamp.Clamp

class SynchronizedArm : PositionArm() {
    private var currentState = ArmState.GO_TARGET

    private var clawTurnStartTime = SystemClock.elapsedRealtime() / 1000.0
    private val clawTurnTime = 0.5 //Seconds needed for claw to turn to position
    private var clawTurning = false

    var currentTargetCoord = ArmPosition.HOME.coordinate

    val clamp = Clamp()


    override fun init(hardwareMap: HardwareMap) {
        super.init(hardwareMap)
        controllers.add(clamp)
        clamp.power = 0.0
    }

    override fun start() {
        super.start()

        clamp.angle = TURN_POS.CLOSED.value
    }

    override fun update(timeStep: Double) {
        super.update(timeStep)

        when (currentState) {
            ArmState.EXCHANGE_FRONT_TO_BACK -> {
//                if (currentCoordinate.closeTo(ArmPosition.EXCHANGE.coordinate)) {
//                    clamp.angle = 180.0

//                }
//                targetCoordinate = ArmPosition.EXCHANGE.coordinate7
                if (currentAngles.lowerAngle + currentAngles.upperAngle >= 0) {
                    clamp.angle = 180.0
                    if (!clawTurning) {
                        upperSpeedLimit = 0.0
                        lowerSpeedLimit = 0.0
                        clawTurning = true
                        clawTurnStartTime = SystemClock.elapsedRealtime() / 1000.0
                    } else if (SystemClock.elapsedRealtime() / 1000.0 - clawTurnStartTime > clawTurnTime) {
                        clawTurning = false
                        currentState = ArmState.GO_TARGET
                    }
                }

            }
            ArmState.EXCHANGE_BACK_TO_FRONT -> {
                upperSpeedLimit = 0.6
                lowerSpeedLimit = 0.4
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
                targetCoordinate = ArmPosition.EXCHANGE.coordinate
            }
            ArmState.GO_TARGET -> {
                upperSpeedLimit = 0.6
                lowerSpeedLimit = 0.4
                targetCoordinate = currentTargetCoord
            }
        }
    }

    override fun moveToPosition(position: ArmPosition) {
        if (position.coordinate != targetCoordinate && !clawTurning) {
            currentTargetCoord = position.coordinate
            if (currentCoordinate.x > 0.0 && position.coordinate.x < 0.0) {
                currentState = ArmState.EXCHANGE_FRONT_TO_BACK
            } else if (currentCoordinate.x < 0.0 && position.coordinate.x > 0.0) {
                currentState = ArmState.EXCHANGE_BACK_TO_FRONT
            }
        }

        super.moveToPosition(position)
    }
}