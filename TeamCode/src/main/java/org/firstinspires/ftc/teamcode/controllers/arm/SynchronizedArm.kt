package org.firstinspires.ftc.teamcode.controllers.arm

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.controllers.clamp.Clamp
import org.firstinspires.ftc.teamcode.core.Coordinate

class SynchronizedArm : PositionArm() {
    private var currentState = ArmState.GO_TARGET

    private var clawTurnStartTime = SystemClock.elapsedRealtime() / 1000.0
    private val clawTurnTime = 0.5 //Seconds needed for claw to turn to position
    private var clawTurning = false

    var currentTargetCoord = ArmPosition.HOME.coordinate

    val clamp = Clamp()

    override fun init(hardwareMap: HardwareMap) {
        super.init(hardwareMap)

        // This shouldn't be here, it should me managed by a helper class.
        clamp.init(hardwareMap)

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
                targetCoordinate = ArmPosition.EXCHANGE.coordinate
            }
            ArmState.EXCHANGE_BACK_TO_FRONT -> {
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