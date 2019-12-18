package org.firstinspires.ftc.teamcode.controllers.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.channels.Channel.Factory.CONFLATED
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.ValueCacher
import org.firstinspires.ftc.teamcode.core.Controller

class EctoDcMotor(val name: String) : Controller() {

    lateinit var motorBase: DcMotorEx

    override fun init(hardwareMap: HardwareMap) {
        motorBase = hardwareMap.get(DcMotor::class.java, name) as DcMotorEx
    }

    // Motor position
    private val positionCacher = ValueCacher(0)
    val currentPosition: Int
        get() = positionCacher.cachedGet {
            motorBase.currentPosition
        }()

    // Motor velocity
    private val velocityChannel = Channel<Double>(CONFLATED)
    private val velocityCacher = ValueCacher(0.0)
    private var lastReceivedVelocity = 0.0
    var velocity: Double
        get() = velocityCacher.cachedGet {
            motorBase.getVelocity(AngleUnit.RADIANS)
        }()
        set(value) = runBlocking {
            if (lastReceivedVelocity != value) {
                velocityChannel.send(value)
                lastReceivedVelocity = value
            }
        }

    // Motor mode
    private val modeChannel = Channel<DcMotor.RunMode>(CONFLATED)
    private val modeCacher = ValueCacher(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    private var lastReceivedMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    var mode: DcMotor.RunMode
        get() = modeCacher.cachedGet {
            motorBase.mode
        }()
        set(value) = runBlocking {
            if (lastReceivedMode != value) {
                modeChannel.send(value)
                lastReceivedMode = value
            }
        }

    // Motor power
    private val powerChannel = Channel<Double>(CONFLATED)
    private val powerCacher = ValueCacher(0.0)
    private var lastReceivedPower = 0.0
    var power: Double
        get() = powerCacher.cachedGet {
            motorBase.power
        }()
        set(value) = runBlocking {
            if (lastReceivedPower != value) {
                powerChannel.send(value)
                lastReceivedPower = value
            }
        }

    // Motor target position
    private val targetPosChannel = Channel<Int>(CONFLATED)
    private val targetPosCacher = ValueCacher(0)
    private var lastReceivedTargetPos = 0
    var targetPosition: Int
        get() = targetPosCacher.cachedGet {
            motorBase.targetPosition
        }()
        set(value) = runBlocking {
            if (lastReceivedTargetPos != value) {
                targetPosChannel.send(value)
                lastReceivedTargetPos = value
            }
        }

    // Motor direction
    private val directionChannel = Channel<DcMotorSimple.Direction>(CONFLATED)
    private val directionCacher = ValueCacher(DcMotorSimple.Direction.FORWARD)
    private var lastReceivedDirection = DcMotorSimple.Direction.FORWARD
    var direction: DcMotorSimple.Direction
        get() = directionCacher.cachedGet {
            motorBase.direction
        }()
        set(value) = runBlocking {
            if (lastReceivedDirection != value) {
                directionChannel.send(value)
                lastReceivedDirection = value
            }
        }


    // Motor zero power behavior
    private val zeroPowerBehaviorChannel = Channel<DcMotor.ZeroPowerBehavior>(CONFLATED)
    private val zeroPowerBehaviorCacher = ValueCacher(DcMotor.ZeroPowerBehavior.BRAKE)
    private var lastReceivedZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    var zeroPowerBehavior: DcMotor.ZeroPowerBehavior
        get() = zeroPowerBehaviorCacher.cachedGet {
            motorBase.zeroPowerBehavior
        }()
        set(value) = runBlocking {
            if (lastReceivedZeroPowerBehavior != value) {
                zeroPowerBehaviorChannel.send(value)
                lastReceivedZeroPowerBehavior = value
            }
        }


    @ExperimentalCoroutinesApi
    override fun update(timeStep: Double) = runBlocking {
        if (!velocityChannel.isEmpty) {
            motorBase.setVelocity(velocityChannel.receive(), AngleUnit.RADIANS)
        }
        if (!modeChannel.isEmpty) {
            motorBase.mode = modeChannel.receive()
        }
        if (!powerChannel.isEmpty) {
            motorBase.power = powerChannel.receive()
        }
        if (!targetPosChannel.isEmpty) {
            motorBase.targetPosition = targetPosChannel.receive()
        }
        if (!directionChannel.isEmpty) {
            motorBase.direction = directionChannel.receive()
        }
        if (!zeroPowerBehaviorChannel.isEmpty) {
            motorBase.zeroPowerBehavior = zeroPowerBehaviorChannel.receive()
        }
    }

}

