package org.firstinspires.ftc.teamcode.pathfinding;


import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.HashSet;

public class Graph {
    String name;
    HashMap<String, GraphNode> nodes;
    HashMap<GraphNode, HashSet<GraphEdge>> edges;

    public Graph(String name) {
        this.name = name;
        this.nodes = new HashMap<String, GraphNode>();
        this.edges = new HashMap<GraphNode, HashSet<GraphEdge> >();
    }

    public GraphNode addNode(String name, double x, double y) {
        GraphNode node = new GraphNode(name, x, y);
        nodes.put( node.name, node );
        edges.put( node, new HashSet<GraphEdge>());
        return node;
    }

    public GraphNode addNode(GraphNode ret) {
        nodes.put(ret.name, ret );
        edges.put( ret, new HashSet<GraphEdge>());
        return ret;
    }

    public void addEdge(GraphEdge edge) {
        edges.get(edge.first) .add(edge);
        edges.get(edge.second) .add(edge.reversed());
    }

    public GraphNode getNode(String name) {
        return nodes.get(name);

    }

    public GraphNode getNode(double x, double y) {
      for(GraphNode node : nodes.values()) {
        if(node.getX() == x && node.getY() == y) {
          return node;
        }
      }
      return null;
    }

    public HashSet<GraphEdge> getNeighbors(GraphNode node) {
        return edges.get(node);

    }

    public void save(String pathname) throws java.io.FileNotFoundException, java.io.IOException {
        File file = new File(pathname);
        FileOutputStream fout = new FileOutputStream(file);
        PrintWriter out = new PrintWriter(fout);
        out.println("digraph " + this.name + "{");
        for(GraphNode node : nodes.values()) {
            out.println("    " + node.getName() + "[");
            out.println("        pos=\"" + node.getX() + "," + node.getY() + "!\"");
            out.println("        label=\"" + node.getName() +  "\"");
            out.println("    ]");
        }
        for(GraphNode node : nodes.values()) {
            for(GraphEdge edge : edges.get(node)) {
                out.println("    " + edge.getFirst().name + " -> " + edge.getSecond().name + ";");
            }
        }



    out.println("}");
    out.flush();
    fout.close();
        //Save the graph to a dot file
    }
}