package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pathfinding.AstarPathFinder;
import org.firstinspires.ftc.teamcode.pathfinding.Graph;
import org.firstinspires.ftc.teamcode.pathfinding.GraphEdge;
import org.firstinspires.ftc.teamcode.pathfinding.GraphNode;
import org.firstinspires.ftc.teamcode.pathfinding.Path;

import java.util.concurrent.TimeUnit;

@Autonomous (name="AutoWiredLeftB", group="Autonomous")
public class AutoWiredLeftB extends AutoLinearAbstract2022 {
    DcMotor armLift1;
    DcMotor armLift2;
    Servo claw;
    Servo clawLift1;
    Servo clawLift2;
    GraphNode currentPos;
    Graph graph;
    AstarPathFinder astar;
    HuskyLens huskyLens;
    private final int READ_PERIOD = 1;


    public void wait(double waitTime) {
        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.reset();
        while (waitTimer.seconds() < waitTime) {
        }

    }

    public void clawClose() {
        clawLift1.setPosition(0.0 / 90.0);
        clawLift2.setPosition(0.0 / 90.0);
        wait(0.5);
    }

    public void clawOpen() {
        clawLift1.setPosition(5.0 / 90.0);
        clawLift2.setPosition(5.0 / 90.0);
        wait(0.5);
    }

    public void armRaise() {
        armLift1.setTargetPosition(2000);
        armLift2.setTargetPosition(2000);
        armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wait(1.5);
    }

    public void armLower() {
        armLift1.setTargetPosition(0);
        armLift2.setTargetPosition(0);
        armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift1.setPower(-0.9);
        armLift2.setPower(-0.9);
        wait(1.5);
    }

    public Graph buildGraph(String graphName) {
        String colNames = "ABCDEF";
        String rowNames = "123456";

        Graph newGraph = new Graph(graphName);
        int itCol;
        int itRow;
        for (itRow = 0; itRow < rowNames.length(); itRow++) {
            for (itCol = 0; itCol < colNames.length(); itCol++) {
                String name = "" + colNames.charAt(itRow) + rowNames.charAt(itCol);

                GraphNode newNode = newGraph.addNode(name, itRow, itCol);
                int itColsLeft;
                int itRowsBelow;
                for (itRowsBelow = 0; itRowsBelow < itRow; itRowsBelow++) {

                    String downName = "" + colNames.charAt(itRowsBelow) + rowNames.charAt(itCol);
                    System.out.println(downName);
                    GraphNode newNode2 = newGraph.getNode(downName);

                    newGraph.addEdge(new GraphEdge(newNode, newNode2));
                }

                for (itColsLeft = 0; itColsLeft < itCol; itColsLeft++) {

                    String leftName = "" + colNames.charAt(itRow) + rowNames.charAt(itColsLeft);
                    System.out.println(leftName);
                    GraphNode newNode2 = newGraph.getNode(leftName);

                    newGraph.addEdge(new GraphEdge(newNode, newNode2));
                }
            }
        }

        return newGraph;
    }

    public void followPath(Path path) {
        for (GraphEdge edge : path.getEdges()) {
            telemetry.addLine("Moving " + edge.getFirst().getName() + " " + edge.getSecond().getName());
            telemetry.addLine("First " + edge.getFirst().getX() + " " + edge.getFirst().getY());
            telemetry.addLine("Second " + edge.getSecond().getX() + " " + edge.getSecond().getY());
            telemetry.addLine("Dist " + edge.getXDist() + " " + edge.getYDist());

            //X and Y are swapped in the hardware map
            if (edge.getYDist() != 0) {
                if (edge.getYDist() < 0) {
                    telemetry.addLine("Moving right " + edge.getYDist());
                    driveTrain.StrafeRightToTarget(edge.getYDist() * 32, 0.25);
                } else {
                    telemetry.addLine("Moving left " + -edge.getYDist());
                    driveTrain.StrafeLeftToTarget(-edge.getYDist() * 32, 0.25);
                }
            } else if (edge.getXDist() != 0) {
                telemetry.addLine("Moving straight " + edge.getXDist());
                driveTrain.goStraightToTarget(edge.getXDist() * -28.5, 0.25);
            }

            telemetry.update();
            wait(0.5);
            while (!driveTrain.isMoveDone(.25)) {
                if (Kill(28)) {
                    break;
                }
            }
        }
    }

    public void moveTo(String nodeName) {
        GraphNode dest = graph.getNode(nodeName);
        Path path = astar.getShortestPath(currentPos, dest);
        followPath(path);
        currentPos = dest;
    }

    public String ScanHusky() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        // String teamobject;
        String teamobject = null;

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();


        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        //  String teamobject = null;
        //  if (!rateLimit.hasExpired()) {

        //}
        rateLimit.reset();

        /*
         * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
         * Block represents the outline of a recognized object along with its ID number.
         * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
         * referenced in the header comment above for more information on IDs and how to
         * assign them to objects.
         *
         * Returns an empty array if no objects are seen.
         */
        int j = 0;
        HuskyLens.Block[] blocks = huskyLens.blocks();

        telemetry.addData("Block count", blocks.length);
        if (blocks.length == 0) {
            telemetry.addData("Block", blocks[j].toString());
            teamobject = blocks[j].toString();
            //return teamobject;
        } else {
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                teamobject = blocks[i].toString();
                //return teamobject;
            }

        }

        telemetry.update();
        //return teamobject;

        return teamobject;

    }

/*public int scanColor() {
        float[] hsvValues = new float[3];
        ElapsedTime colorTimer = new ElapsedTime();

        //Initialize the color sensor
        NormalizedColorSensor colorSensor;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        int[] colorsFound = new int[3];
        colorsFound[0] = 0;
        colorsFound[1] = 0;
        colorsFound[2] = 0;
        int count = 0;
        colorTimer.reset();
        while (colorTimer.seconds() < 0.5){ // Read the color for half a second
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            //Convert the color from RGBA (Red, Green, Blue, Alpha) to HSV (Hue, Saturation, Value) format
            Color.colorToHSV(colors.toColor(), hsvValues);

            //Check the Hue of the color, count the number of times each color gets read
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
                //Debug code so we can see what color values are being read
                telemetry.addLine("" + hsvValues[0] + "," + hsvValues[1] + "," + hsvValues[2]);
                telemetry.addLine("" + colorsFound[0] + "," + colorsFound[1] + "," + colorsFound[2]);
                telemetry.update();

                wait(0.1);
            }

            if (Kill(28)) {
                break;
            }
        }

        // Figure out which color was read most often
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

        // Show the final color read
        ArrayList<String> colorNames = new ArrayList<String>();
        colorNames.add("Cyan");
        colorNames.add("Magenta");
        colorNames.add("Yellow");

        telemetry.addLine("" + hsvValues[0] + "," + hsvValues[1] + "," + hsvValues[2]);
        telemetry.addLine("" + colorsFound[0] + "," + colorsFound[1] + "," + colorsFound[2]);
        telemetry.addLine("Found Color: " + colorNames.get(bestColor) + "/" + bestColor);
        telemetry.update();

        return bestColor;
    }*/

    public void runOpMode() {
        super.runOpMode();
        telemetry.addLine("Telemetry Initialized");
        telemetry.update();

        wait(0.5);

        graph = buildGraph("Olivia");
        {
            GraphNode start = graph.addNode("start", -0.1, 1);
            GraphNode B4 = graph.getNode("sB4");
            graph.addEdge(new GraphEdge(start, B4));
            GraphNode pixelDrop = graph.addNode("pixelDrop", -0.1, 1);
            graph.addEdge(new GraphEdge(B4, pixelDrop));
        }
        astar = new AstarPathFinder(graph);
        GraphNode start = graph.getNode("start");
        GraphNode pixelDrop = graph.getNode("pixelDrop");
        currentPos = start;

        armLift1 = hardwareMap.get(DcMotor.class, "armLift1");
        armLift2 = hardwareMap.get(DcMotor.class,"armLift2");
        claw=hardwareMap.get(Servo.class, "claw");
        clawLift1 = hardwareMap.get(Servo.class, "clawLift1");
        clawLift2=hardwareMap.get(Servo.class,"clawLift2");

        //Pickup and drop cone

        moveTo("B4");
        wait(2.0);
        driveTrain.RotateLeft(12,25);
        wait(2.0);
        moveTo("pixelDrop");
        //if (false) {

        //Lift the arm

        //Scan the signal cone
                /*String ObjectD1 = null;

                {
                    ObjectD1 = ScanHusky();
                }

                //Raise the arm
                int spikeNumber=0;
                //Move to the right position based on the color scanned
                {
                    if (ObjectD1!=null) {
                        spikeNumber=1;
                        //Scaning middle spike mark team object
                        moveTo("B4");
                        clawLift2.setPosition(1);
                        driveTrain.RotateLeft(12,25);
                        moveTo("B5");


                    } else if (ObjectD1 ==null) {

                        //Scaning Left spike mark team object
                        moveTo("B2");
                    } else {
                        moveTo("B1");
                    }
                }

                //Lower the arm
                {
                    armLower();
                }*/

    }
}
