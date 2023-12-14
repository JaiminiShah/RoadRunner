package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pathfinding.AstarPathFinder;
import org.firstinspires.ftc.teamcode.pathfinding.Graph;
import org.firstinspires.ftc.teamcode.pathfinding.GraphEdge;
import org.firstinspires.ftc.teamcode.pathfinding.GraphNode;
import org.firstinspires.ftc.teamcode.pathfinding.Path;

import java.util.ArrayList;

@Autonomous (name="AutoWired1", group="Autonomous")
public class AutoWired1 extends AutoLinearAbstract2022 {
    DcMotor armLift;
    Servo clawLift;
    GraphNode currentPos;
    Graph graph;
    AstarPathFinder astar;

    public void wait(double waitTime)
    {
        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.reset();
        while (waitTimer.seconds()<waitTime){
        }

    }

    public void clawClose()
    {
        clawLift.setPosition(0.0 / 90.0);
        wait(0.5);
    }

    public void clawOpen()
    {
        clawLift.setPosition(5.0 / 90.0);
        wait(0.5);
    }

    public void armRaise()
    {
        armLift.setTargetPosition(2000);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(0.9);
        wait(1.5);
    }

    public void armLower()
    {
        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(-0.9);
        wait(1.5);
    }

    public Graph buildGraph(String graphName){
        String colNames = "ABCDEF";
        String rowNames = "123456";

        Graph newGraph = new Graph(graphName);
        int itCol;
        int itRow;
        for (itRow = 0; itRow < rowNames.length(); itRow++) {
            for (itCol = 0; itCol < colNames.length(); itCol++) {
                String name = "s" + colNames.charAt(itRow) + rowNames.charAt(itCol);

                GraphNode newNode = newGraph.addNode(name, itRow, itCol);
                int itColsLeft;
                int itRowsBelow;
                for(itRowsBelow = 0; itRowsBelow < itRow; itRowsBelow++) {

                    String downName = "s" + colNames.charAt(itRowsBelow) + rowNames.charAt(itCol);
                    System.out.println(downName);
                    GraphNode newNode2 = newGraph.getNode(downName);

                    newGraph.addEdge(new GraphEdge(newNode, newNode2));
                }

                for(itColsLeft = 0; itColsLeft < itCol; itColsLeft++) {

                    String leftName = "s" + colNames.charAt(itRow) + rowNames.charAt(itColsLeft);
                    System.out.println(leftName);
                    GraphNode newNode2 = newGraph.getNode(leftName);

                    newGraph.addEdge(new GraphEdge(newNode, newNode2));
                }
            }
        }

        return newGraph;
    }

    public void followPath(Path path) {
        for(GraphEdge edge : path.getEdges()) {
            telemetry.addLine("Moving " + edge.getFirst().getName() + " " + edge.getSecond().getName());
            telemetry.addLine("First " + edge.getFirst().getX() + " " + edge.getFirst().getY());
            telemetry.addLine("Second " + edge.getSecond().getX() + " " + edge.getSecond().getY());
            telemetry.addLine("Dist " + edge.getXDist() + " " + edge.getYDist());

            //X and Y are swapped in the hardware map
            if(edge.getYDist() != 0) {
                if(edge.getYDist() < 0) {
                    telemetry.addLine("Moving right " + edge.getYDist());
                    driveTrain.StrafeRightToTarget(edge.getYDist() * 32, 0.25);
                }else{
                    telemetry.addLine("Moving left " + -edge.getYDist());
                    driveTrain.StrafeLeftToTarget(-edge.getYDist() * 32, 0.25);
                }
            }else if(edge.getXDist() != 0) {
                telemetry.addLine("Moving straight " + edge.getXDist());
                driveTrain.goStraightToTarget(edge.getXDist() * -28.5, 0.25);
            }
            telemetry.update();
            while (!driveTrain.isMoveDone(.25)) {
                if (Kill(28)) {
                    break;
                }
            }
        }
    }

    public void moveTo(String nodeName) {
        GraphNode dest = graph.getNode("s" + nodeName);
        Path path = astar.getShortestPath(currentPos, dest);
        followPath(path);
        currentPos = dest;
    }

    public int scanColor() {
        NormalizedColorSensor colorSensor;
        float[] hsvValues = new float[3];
        ElapsedTime colorTimer = new ElapsedTime();
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        int[] colorsFound = new int[3];
        colorsFound[0] = 0;
        colorsFound[1] = 0;
        colorsFound[2] = 0;
        int count = 0;
        colorTimer.reset();
        while (colorTimer.seconds()<0.5){
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            if (hsvValues[0] > 190 && hsvValues[0] < 260) {
                //Cyan/Blue
                colorsFound[0] ++;
            }
            else if(hsvValues[0] > 300 && hsvValues[0] < 360){
                //Magenta/Pink
                colorsFound[1] ++;
            }
            else if(hsvValues[0] > 70 && hsvValues[0] < 130){
                //Yellow/Green
                colorsFound[2] ++;
            }

            if(false) {
                telemetry.addLine("" + hsvValues[0] + "," + hsvValues[1] + "," + hsvValues[2]);
                telemetry.addLine("" + colorsFound[0] + "," + colorsFound[1] + "," + colorsFound[2]);
                telemetry.update();

                wait(0.1);
            }

            if (Kill(28)) {
                break;
            }
        }

        int bestColor = 0;
        if(colorsFound[0]<colorsFound[1]) {
            if(colorsFound[1]<colorsFound[2]) {
                bestColor = 2;
            }else{
                bestColor = 1;
            }
        }else{
            if(colorsFound[0]<colorsFound[2]) {
                bestColor = 2;
            }else{
                bestColor = 0;
            }
        }

        ArrayList<String> colorNames = new ArrayList<String>();

        colorNames.add("Cyan");
        colorNames.add("Magenta");
        colorNames.add("Yellow");

        telemetry.addLine("" + hsvValues[0] + "," + hsvValues[1] + "," + hsvValues[2]);
        telemetry.addLine("" + colorsFound[0] + "," + colorsFound[1] + "," + colorsFound[2]);
        telemetry.addLine("Found Color: " + colorNames.get(bestColor) + "/" + bestColor);
        telemetry.update();

        return bestColor;
    }

    @Override
    public void runOpMode(){
        super.runOpMode();
        telemetry.addLine("Telemetry Initialized");
        telemetry.update();

        wait(0.5);

        graph = buildGraph("Olivia");
        {
            GraphNode start = graph.addNode("start", -0.1, 1);
            GraphNode rightLow = graph.addNode("rightLow", 0.4, 1.5);
            GraphNode A1 = graph.getNode("sA1");
            graph.addEdge(new GraphEdge(start, A1));
            GraphNode B2 = graph.getNode("sB2");
            graph.addEdge(new GraphEdge(start, B2));
        }
        astar = new AstarPathFinder(graph);
        GraphNode S = graph.getNode("start");
        GraphNode A1 = graph.getNode("sA1");
        currentPos = S;

        armLift = hardwareMap.get(DcMotor.class, "armlift");
        clawLift = hardwareMap.get(Servo.class, "clawlift");

        //Pickup and drop cone
        {
            clawClose();
            armRaise();
            wait(2.0);
            clawOpen();
            armLower();
        }

        moveTo("right_low");

        if(false) {

            //Lift the arm
            {
                armRaise();
                clawOpen();
            }

            //Scan the signal cone
            int bestColor = 0;

            {
                moveTo("B2");
                armLower();
                bestColor = scanColor();
            }

            //Raise the arm
            {
                armRaise();
            }

            //Move to the right position based on the color scanned
            {
                if (bestColor == 0) {
                    moveTo("B3");
                } else if (bestColor == 1) {
                    moveTo("B2");
                } else {
                    moveTo("B1");
                }
            }

            //Lower the arm
            {
                armLower();
            }
        }
    }
}