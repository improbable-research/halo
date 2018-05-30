import io.improbable.keanu.vertices.dbl.DoubleVertex

class Vector3D (var x: DoubleVertex, var y: DoubleVertex, var z: DoubleVertex) {

    operator fun plus(other: Vector3D): Vector3D {
        return Vector3D(x + other.x, y + other.y, z + other.z)
    }

    operator fun minus(other: Vector3D): Vector3D {
        return Vector3D(x - other.x, y - other.y, z - other.z)
    }

    fun dot(other: Vector3D): DoubleVertex {
        return x * other.x + y * other.y + z * other.z
    }

    fun cross(other: Vector3D): Vector3D {
        return Vector3D(y * other.z - z * other.y,
                        z * other.x - x * other.z,
                        x * other.y - y * other.x)
    }

    fun lengthSquared(): DoubleVertex {
        return (x * x + y * y + z * z)
    }

    fun length(): DoubleVertex {
        return  lengthSquared().pow(0.5)
    }

    operator fun div(denominator: DoubleVertex): Vector3D {
        return Vector3D(x / denominator, y / denominator, z / denominator)
    }

    operator fun div(denominator: Double): Vector3D {
        return Vector3D(x / denominator, y / denominator, z / denominator)
    }

    fun unit(): Vector3D {
        return this / length()
    }
}