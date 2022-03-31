package frc.robot.util

class Scale(scale: Double) {

    var scale: Double = scale
        private set

    companion object {
        @JvmStatic
        fun from(vararg scales: Double): Scale {
            if (scales.isEmpty()) {
                return Scale(0.0)
            }
            var netScale = 1.0
            for (scale in scales) {
                netScale *= scale;
            }
            return Scale(netScale)
        }

        @JvmStatic
        @JvmOverloads
        fun from(scale: Double = 1.0, vararg otherScales: Scale): Scale {
            var netScale: Double = scale;
            for (otherScale: Scale in otherScales) {
                netScale *= otherScale.scale
            }
            return Scale(netScale)
        }
    }

    fun scale(number: Double): Double {
        return number * scale
    }

    fun addScale(number: Double) {
        scale *= number
    }

}