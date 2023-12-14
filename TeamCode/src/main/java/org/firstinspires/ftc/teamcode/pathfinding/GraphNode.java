package org.firstinspires.ftc.teamcode.pathfinding;

import java.io.PrintWriter;
import java.io.StringWriter;

public class GraphNode {
    protected String name, label;
    protected double x, y;

    GraphNode(String name, double x, double y) {
        this.name = name;
        this.label = name;
        this.x = x;
        this.y = y;
    }

    public String getName() {
        return name;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getDistance(GraphNode node) {
        return Math.abs(node.x - x) + Math.abs(node.y - y);
    }

    @Override
    public String toString() {
        StringWriter writer = new StringWriter();
        PrintWriter out = new PrintWriter(writer);

        out.println("    " + this.name + " [");
        out.println("        pos=\"" + this.x + "," + this.y + "\"");
        out.println("        label=\"" + this.label + "\"");
        out.println("    ]");

        return writer.toString();
    }
}
