package com.symmetry.furo.physics

import com.symmetry.furo.extensions.plus
import com.symmetry.furo.extensions.times
import org.ojalgo.matrix.Primitive64Matrix

interface NonLinearDynamicModel : DynamicModel {
    fun stateTransitionMatrix(state: Primitive64Matrix, dt: Double): Primitive64Matrix
    fun inputTransitionMatrix(state: Primitive64Matrix, dt: Double): Primitive64Matrix

    override fun simulate(state: Primitive64Matrix, input: Primitive64Matrix, dt: Double): Primitive64Matrix {
        return (stateTransitionMatrix(state, dt) * state) + (inputTransitionMatrix(state, dt) * input)
    }
}

