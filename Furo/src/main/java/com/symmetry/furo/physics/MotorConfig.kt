package com.symmetry.furo.physics

val DEFAULT_BACKDRIVE_TORQUE_PER_GEARING = 0.01694772439999992 / 3.7;

enum class MotorConfig(
    val rpm: Double,
    val encoderTicksPerRevolution: Double,
    val stallTorque: Double,
    val stallCurrent: Double,
    val freeCurrent: Double,
    val backdriveTorque: Double
) {
    GOBILDA_1150_RPM(
        1150.0, 145.6, 0.7697091498333297,
        9.2, 0.25, DEFAULT_BACKDRIVE_TORQUE_PER_GEARING * 5.2
    ),
    GOBILDA_435_RPM(
        435.0, 383.6, 1.836003476666658,
        9.2, 0.25, DEFAULT_BACKDRIVE_TORQUE_PER_GEARING * 13.7
    ),
    GOBILDA_312_RPM(
        312.0, 537.6, 2.3868045196666556,
        9.2, 0.25, DEFAULT_BACKDRIVE_TORQUE_PER_GEARING * 19.2
    ),
    GOBILDA_223_RPM(
        223.0, 753.2, 3.742622471666649,
        9.2, 0.25, DEFAULT_BACKDRIVE_TORQUE_PER_GEARING * 26.9
    ),
    NEVEREST_3_7(
        1780.0, 103.6, 0.17 * 3.7,
        9.801, 0.355, DEFAULT_BACKDRIVE_TORQUE_PER_GEARING * 3.7
    ),
    NEVEREST_20(
        340.0, 537.6, 0.17 * 20,
        9.801, 0.355, DEFAULT_BACKDRIVE_TORQUE_PER_GEARING * 20
    ),
    REV_CORE_HEX(
        125.0, 288.0, 3.2,
        4.4, 0.0, DEFAULT_BACKDRIVE_TORQUE_PER_GEARING * 72
    );

    fun getMaxAngularVelocity(): Double {
        return rpm * Math.PI / 30.0
    }
}