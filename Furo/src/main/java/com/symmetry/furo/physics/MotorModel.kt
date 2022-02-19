package com.symmetry.furo.physics

import java.util.function.DoubleUnaryOperator

data class MotorModel(
    val gearRatio: Double,
    val nominalVoltage: Double,
    var stallTorque: Double,
    val stallCurrent: Double,
    val freeCurrent: Double,
    var freeSpeed: Double,
    val efficiency: Double,
    val inertia: DoubleUnaryOperator,
    val weightAppliedTorque: DoubleUnaryOperator?,
    val staticFriction: Double,
    val coulombFriction: Double,
    val viscousFriction: Double,
    val stribeckPower: Double,
    val stribeckVelocity: Double
) {
    val RPM_TO_RAD_PER_SECOND: Double = Math.PI / 30.0;

    init {
        stallTorque *= gearRatio * efficiency;
        freeSpeed *= RPM_TO_RAD_PER_SECOND
    }

    private var currentAngularPosition: Double = 0.0;
    private var currentAngularVelocity: Double = 0.0;
    private var lastAngularAcceleration: Double = 0.0;

    val resistance = nominalVoltage / stallCurrent
    val kV = (nominalVoltage - resistance * freeCurrent) / freeSpeed
    val kT = stallTorque / stallCurrent


    fun generateMotorModel(
        motorType: MotorConfig,
        motorCount: Int = 1,
        externalGearRatio: Double = 1.0,
        inertia: DoubleUnaryOperator = DoubleUnaryOperator { motorType.backdriveTorque },
        weightAppliedTorque: DoubleUnaryOperator?
    ): MotorModel {
        return MotorModel(
            externalGearRatio,
            12.0,
            motorType.stallTorque * motorCount,
            motorType.stallCurrent,
            motorType.freeCurrent,
            motorType.rpm,
            1.0,
            inertia,
            weightAppliedTorque,
            3E-3 * motorCount,
            2E-3 * motorCount,
            1E-4,
            0.05, 25.0
        )
    }

    fun update(dt: Double, voltageInput: Double, externalFriction: Double = 0.0) {
        if (dt == 0.0) {
            return
        }

        val voltageInput = if (voltageInput > nominalVoltage) nominalVoltage else if (voltageInput < -nominalVoltage) -nominalVoltage else voltageInput
        lastAngularAcceleration = calculateAngularAcceleration(voltageInput, externalFriction)
        currentAngularVelocity += lastAngularAcceleration * dt
        currentAngularPosition += currentAngularVelocity * dt

        if (currentAngularPosition < 0.0) {
            currentAngularPosition = 0.0
        }
    }

    fun calculateTorque(voltageInput: Double, externalFriction: Double): Double {
        val torque: Double = kT * efficiency * gearRatio * (voltageInput - kV * currentAngularVelocity * gearRatio) / resistance
        val frictionTorque: Double = getFrictionTorque() + externalFriction;

        if (Math.abs(Math.signum(torque) - Math.signum(torque - frictionTorque)) == 2.0) {
            return 0.0
        } else {
            return torque - frictionTorque - (weightAppliedTorque?.applyAsDouble(currentAngularPosition) ?: 0.0)
        }
    }

    fun calculateAngularAcceleration(voltageInput: Double, externalFriction: Double): Double {
        return calculateTorque(voltageInput, externalFriction) / inertia.applyAsDouble(currentAngularPosition)
    }


    fun getFrictionTorque(): Double {
        return if (Math.signum(currentAngularVelocity) != 0.0) Math.signum(currentAngularVelocity) * (coulombFriction + (staticFriction - coulombFriction) * Math.exp(
            -Math.pow(
                Math.abs(
                    currentAngularVelocity /
                            stribeckVelocity
                ), stribeckPower
            )
        ) + viscousFriction * Math.abs(currentAngularVelocity)) else Math.signum(staticFriction) * staticFriction
    }

    fun getLinearPosition(currentAngularPosition: Double, rotationDiameter: Double): Double {
        return currentAngularPosition * rotationDiameter / 2.0
    }

    fun getLinearPosition(rotationDiameter: Double): Double {
        return currentAngularPosition * rotationDiameter / 2.0
    }

    fun getLinearVelocity(rotationDiameter: Double): Double {
        return currentAngularVelocity * rotationDiameter / 2.0
    }

    fun getLinearAcceleration(rotationDiameter: Double): Double {
        return lastAngularAcceleration * rotationDiameter / 2.0
    }

    override fun toString(): String {
        return currentAngularPosition.toString() + "\t" + currentAngularVelocity + "\t" + lastAngularAcceleration
    }


    companion object {
        @JvmStatic
        fun main(args: Array<String>) {
            val motorModel = MotorModel(
                20.0, 12.0, 3.36, 166.0,
                1.3, 5880.0, 0.8, { 2.0 }, { 0.0 },
                3E-3, 2E-3, 1E-4, 0.05, 25.0
            )
            println("Time\tθ\tω\tα")
            val dt = 0.001
            for (i in 1..499) {
                motorModel.update(dt, 12.0)
                print(((dt * i * 1000) / 1000).toString() + "\t")
                println(motorModel)
            }
        }
    }
}