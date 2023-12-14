package org.firstinspires.ftc.teamcode.pathfinding;

import java.util.Comparator;
import java.util.HashMap;
import java.util.PriorityQueue;

public class AstarPathFinder {
  class EstimatedCostComparator implements Comparator<Path>
  {
    GraphNode destination;

    EstimatedCostComparator(GraphNode destination) {
      this.destination = destination;
    }

    public int compare(Path a, Path b)
    {
      double aCost = a.estimateCostTo(destination);
      double bCost = b.estimateCostTo(destination);

      if (aCost < bCost) {
        return -1;
      } else if (aCost > bCost) {
        return 1;
      } else {
        return 0;
      }
    }
  }

  Graph graph;

  public AstarPathFinder(Graph graph) {
    this.graph = graph;
  }

  public Path getShortestPath(GraphNode start, GraphNode end) {
    if (start == end) {
      return new Path();
    }
    EstimatedCostComparator comparator = new EstimatedCostComparator(end);
    PriorityQueue<Path> pathList = new PriorityQueue<>(20, comparator);

    Path shortestPath = null;

    for(GraphEdge edge: graph.getNeighbors(start))
    {
      Path newPath = new Path().addEdge(edge);
      pathList.add(newPath);
      if(newPath.getFinalNode() == end) {
        if(shortestPath == null || newPath.getTotalCost() < shortestPath.getTotalCost())
          shortestPath = newPath;
      }
    }
    System.out.println("Initial Path");
    for(Path path : pathList) {
      System.out.println("Initial:" + path.toString() + " " + path.getTotalCost() + " " + path.estimateCostTo(end));
    }

    HashMap<GraphNode, Double> searchNodes = new HashMap<GraphNode, Double>();
    searchNodes.put(start, 0.0);

    while(!pathList.isEmpty()) {
      Path path = pathList.poll();

      System.out.println("Searching: " + path.toString());

      if(shortestPath != null && path.estimateCostTo(end) >= shortestPath.getTotalCost()) {
        break;
      }
      for(GraphEdge edge: graph.getNeighbors(path.getFinalNode())) {
        Path newPath = new Path(path).addEdge(edge);


        {
          Double prevCostOrNull = searchNodes.get(edge.getSecond());
          if(prevCostOrNull == null) {
            searchNodes.put(edge.getSecond(), newPath.getTotalCost());
          }else if(newPath.getTotalCost() < prevCostOrNull) {
            searchNodes.put(edge.getSecond(), newPath.getTotalCost());
          }else{
            continue;
          }
        }

        if(edge.getSecond() == end && shortestPath == null) {
          shortestPath = newPath;
        }else if(edge.getSecond() == end && newPath.getTotalCost() < shortestPath.getTotalCost()) {
          shortestPath = newPath;
        }else if(edge.getSecond() != end) {
          pathList.add(newPath);
        }
      }
    }
    System.out.println("AstarPathFinder.java is successful");
    return shortestPath;
  }
}
