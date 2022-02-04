package com.symmetry.furo.physics

import org.ojalgo.matrix.Primitive64Matrix

interface DynamicModel {
    fun simulate(state: Primitive64Matrix, input: Primitive64Matrix, dt: Double): Primitive64Matrix
}