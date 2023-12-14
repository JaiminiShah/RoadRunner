package org.firstinspires.ftc.teamcode.pathfinding;

public class GraphEdge {

    GraphNode first;
    GraphNode second;

    public GraphEdge(GraphNode first, GraphNode second) {
        this.first = first;
        this.second = second;
    }

    public GraphNode getFirst() {
        return first;
    }

    public GraphNode getSecond() {
        return second;
    }

    public GraphEdge reversed() {
        return new GraphEdge(this.second, this.first);
    }

    public double getXDist() {
        return this.second.getX() - this.first.getX();
    }

    public double getYDist() {
        return this.second.getY() - this.first.getY();
    }

    public double getCost() {
        return 0.01 + Math.abs(this.first.getX() - this.second.getX()) + Math.abs(this.first.getY() - this.second.getY());
    }
}
