package com.symmetry.furo

import org.ojalgo.matrix.Primitive64Matrix


class Main {
    companion object {
        @JvmStatic
        fun main(args: Array<String>) {
            val p = Primitive64Matrix.FACTORY;
            val m = p.make(5, 6)
            println(m)

        }
    }
}