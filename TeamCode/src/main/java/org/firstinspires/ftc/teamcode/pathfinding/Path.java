package org.firstinspires.ftc.teamcode.pathfinding;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Collection;
import java.util.LinkedList;

public class Path {
    LinkedList<GraphEdge> edges;
    double totalCost;

    public Path() {
        // This constructor creates an empty Path
        edges = new LinkedList<GraphEdge>();
        totalCost = 0.0;
    }

    Path(Collection<GraphEdge> edges) {
        // This is a constructor that creates a new Path from a
        // list of edges.  
       this.edges = new LinkedList<GraphEdge>();
        this.edges.addAll(edges);
        totalCost = 0.0;
        for(GraphEdge edge : this.edges) {
            totalCost += edge.getCost();
        }
    }

    public Path(Path path) {
        // This is a constructor that creates a copy of path
        edges = new LinkedList<GraphEdge>();
        edges.addAll(path.getEdges());
        totalCost = path.getTotalCost();
    }

    public LinkedList<GraphEdge> getEdges() {
        // Return the list of edges
        return edges;
    }

    public Path addEdge(GraphEdge edge) {
        // Add an edge to the end of the path
        edges.add(edge);
        totalCost += edge.getCost();
        return this;
    }

    public double getTotalCost() {
        // Returns the cost of traversing the path
        return totalCost;
    }

    public GraphNode getFinalNode() {
        // Returns the last node on the path
        return edges.getLast().getSecond();
    }


    public double estimateCostTo(GraphNode destination) {
        // Estimates the whole cost of getting from 
        // the last node in the path to the destination
        //
        // We'll need this for A-Star, but ignore it for now
        GraphNode finalnode = getFinalNode();
        double ret = getTotalCost() + finalnode.getDistance(destination);
        if (finalnode.getX() != destination.getX()) {
            ret += 0.01;
        }
        if(finalnode.getY() != destination.getY()) {
            ret += 0.01;
        }
        return ret;
    }



    public int size() {
        // Returns the number of edges in the path
        return edges.size();
    }


    public String toString() {
        StringWriter writer = new StringWriter();
        PrintWriter out = new PrintWriter(writer);

        if(edges.size() == 0) {
            return "!EMPTY!";
        }

        GraphEdge firstEdge = edges.getFirst();
        GraphNode startingNode = firstEdge.getFirst();
        out.print(startingNode.getName());
        for(GraphEdge edge : edges) {
            out.print("=>" + edge.getSecond().getName());
        }

        out.flush();
        return writer.toString();
    }
}
