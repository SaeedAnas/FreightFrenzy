package com.symmetry.furo.extensions

import org.ojalgo.matrix.Primitive64Matrix

operator fun Primitive64Matrix.plus(matrix: Primitive64Matrix): Primitive64Matrix {
    return this.add(matrix)
}

operator fun Primitive64Matrix.times(matrix: Primitive64Matrix): Primitive64Matrix {
    return this.multiply(matrix)
}

operator fun Primitive64Matrix.minus(matrix: Primitive64Matrix): Primitive64Matrix {
    return this.subtract(matrix)
}