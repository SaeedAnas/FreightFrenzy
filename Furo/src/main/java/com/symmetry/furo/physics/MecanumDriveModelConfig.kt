package com.symmetry.furo.physics

data class MecanumDriveModelConfig(
    val dt: Double,
    val robotMass: Double,
    val wheelMass: Double,
    val wheelInertiaSpinning: Double,
    val wheelInertiaTurning: Double,
    val robotMomentInertia: Double,
    val wheelRadius: Double,
    val L1: Double,
    val L2: Double,
    val D1: Double,
    val D2: Double,
    val motorModel: MotorModel
)