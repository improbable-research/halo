import io.improbable.keanu.vertices.dbl.DoubleVertex

class ProbabilisticVector3D (var x: DoubleVertex, var y: DoubleVertex, var z: DoubleVertex) {

    operator fun plus(other: ProbabilisticVector3D): ProbabilisticVector3D {
        return ProbabilisticVector3D(x + other.x, y + other.y, z + other.z)
    }

    operator fun minus(other: ProbabilisticVector3D): ProbabilisticVector3D {
        return ProbabilisticVector3D(x - other.x, y - other.y, z - other.z)
    }

    fun dot(other: ProbabilisticVector3D): DoubleVertex {
        return x * other.x + y * other.y + z * other.z
    }

    fun cross(other: ProbabilisticVector3D): ProbabilisticVector3D {
        return ProbabilisticVector3D(y * other.z - z * other.y,
                        z * other.x - x * other.z,
                        x * other.y - y * other.x)
    }

    fun lengthSquared(): DoubleVertex {
        return (x * x + y * y + z * z)
    }

    fun length(): DoubleVertex {
        return  lengthSquared().pow(0.5)
    }

    operator fun div(denominator: DoubleVertex): ProbabilisticVector3D {
        return ProbabilisticVector3D(x / denominator, y / denominator, z / denominator)
    }

    operator fun div(denominator: Double): ProbabilisticVector3D {
        return ProbabilisticVector3D(x / denominator, y / denominator, z / denominator)
    }

    operator fun times(scalar: DoubleVertex): ProbabilisticVector3D {
        return ProbabilisticVector3D(x * scalar, y * scalar, z * scalar)
    }

    operator fun times(scalar: Double): ProbabilisticVector3D {
        return ProbabilisticVector3D(x * scalar, y * scalar, z * scalar)
    }

    fun unit(): ProbabilisticVector3D {
        return this / length()
    }
}