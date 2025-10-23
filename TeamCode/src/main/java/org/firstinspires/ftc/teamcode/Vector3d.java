package org.firstinspires.ftc.teamcode;

public class Vector3d {
    public double x, y, z;

    public Vector3d() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    // Constructor có tham số
    public Vector3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3d add(Vector3d v) {
        return new Vector3d(x + v.x, y + v.y, z + v.z);
    }

    public Vector3d set(Vector3d a){
        return new Vector3d(a.x, a.y, a.z);
    }

    public Vector3d minus(Vector3d v) {
        return new Vector3d(x - v.x, y - v.y, z - v.z);
    }

    public Vector3d times(double scalar) {
        return new Vector3d(x * scalar, y * scalar, z * scalar);
    }

    public double dot(Vector3d v) {
        return x * v.x + y * v.y + z * v.z;
    }

    public Vector3d cross(Vector3d v) {
        return new Vector3d(
                y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x
        );
    }

    public double norm() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public Vector3d normalized() {
        double len = norm();
        return len == 0 ? new Vector3d(0, 0, 0) : new Vector3d(x / len, y / len, z / len);
    }

    public double distanceTo(Vector3d v) {
        return Math.sqrt(
                Math.pow(x - v.x, 2) +
                        Math.pow(y - v.y, 2) +
                        Math.pow(z - v.z, 2)
        );
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f, %.2f)", x, y, z);
    }
}