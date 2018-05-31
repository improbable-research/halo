import org.apache.commons.math3.geometry.euclidean.threed.Vector3D

object Geometry {
    fun cartesianToSpherical(xyz : Vector3D) : Vector3D {
        val unitNorm = xyz.normalize()
        return Vector3D(
                xyz.norm,
                Math.acos(1.0-xyz.y),
                Math.atan(xyz.z/xyz.x)
        )
    }

    fun erectToFlacid(spherical : Vector3D) : Vector3D {
        return Vector3D(
                spherical.x,
                -spherical.y,
                spherical.z
        )
    }

    fun sphericalToCartesian(spherical : Vector3D) : Vector3D {
        return Vector3D(
                Math.sin(spherical.y) * Math.cos(spherical.z),
                Math.cos(spherical.y),
                Math.sin(spherical.y) * Math.sin(spherical.z)
        )
    }

}