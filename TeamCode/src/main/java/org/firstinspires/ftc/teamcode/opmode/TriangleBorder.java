package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class TriangleBorder {
    public static List<Vector2d> bresenham(Vector2d start, Vector2d end) {
        List<Vector2d> points = new ArrayList<>();
        double x1 = start.x, y1 = start.y;
        double x2 = end.x, y2 = end.y;

        double dx = Math.abs(x2 - x1);
        double dy = Math.abs(y2 - y1);
        double sx = x1 < x2 ? 1 : -1;
        double sy = y1 < y2 ? 1 : -1;
        double err = dx - dy;

        while (true) {
            points.add(new Vector2d(x1, y1));
            if (x1 == x2 && y1 == y2) break;
            double e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y1 += sy;
            }
        }
        return points;
    }

    public static Set<Vector2d> triangleBorder(Vector2d A, Vector2d B, Vector2d C) {
        Set<Vector2d> border = new HashSet<>();
        border.addAll(bresenham(A, B));
        border.addAll(bresenham(B, C));
        border.addAll(bresenham(C, A));
        return border;
    }

    public static Vector2d GetNearstPos(Vector2d roboPos){
        Vector2d A = new Vector2d(-40, 25);
        Vector2d B = new Vector2d(0, 0);
        Vector2d C = new Vector2d(-40, -25);

        Set<Vector2d> border = triangleBorder(A, B, C);
        Vector2d result = new Vector2d(10000, 10000);
        for (Vector2d p : border) {
            System.out.println(p + " " + Pytagoras(p, roboPos));
            if(Pytagoras(p, roboPos) < Pytagoras(result, roboPos)){
                result = p;
            }
        }
        return result;
    }

    public static double Pytagoras(Vector2d v1, Vector2d v2) {
        double dx = v1.x - v2.x;
        double dy = v1.y - v2.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
}
