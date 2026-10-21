package swarmdrone.decentralizedswarmcoverage_ijrcs;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Orientation;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.image.Image;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.ScrollEvent;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.shape.SVGPath;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;

import java.io.File;
import java.util.Locale;

public class MainView extends Application {

    // BUTTON
    private final ToggleGroup toolGroup = new ToggleGroup();
    private final Button btnNew = initializeButton(SVGIcon.newEnv, 1.0, 1.0, "New Environment");// VISUALIZATION
    private final Button btnOpen = initializeButton(SVGIcon.openEnv, 1.0, 1.0, "Open Environment");// VISUALIZATION
    private final Button btnSave = initializeButton(SVGIcon.saveEnv, 1.0, 1.0, "Save Environment");// VISUALIZATION
    private final ToggleButton btnTransform = initializeToggleButton(SVGIcon.transform, 1.0, 1.0, "Transform");// VISUALIZATION
    private final ToggleButton btnAddDrone = initializeToggleButton(SVGIcon.addDrone, 1.0, 1.0, "Add Drone");// VISUALIZATION
    private final ToggleButton btnRemoveDrone = initializeToggleButton(SVGIcon.removeDrone, 1.0, 1.0, "Remove Drone");// VISUALIZATION
    private final ToggleButton btnWall = initializeToggleButton(SVGIcon.wall, 1.0, 1.0, "Wall");// VISUALIZATION
    private final ToggleButton btnDrawLine = initializeToggleButton(SVGIcon.drawLine, 1.0, 1.0, "Draw Line");// VISUALIZATION
    private final ToggleButton btnRemoveLine = initializeToggleButton(SVGIcon.removeLine, 1.0, 1.0, "Remove Line");// VISUALIZATION
    private final Button btnRun = initializeButton(SVGIcon.run, 1.0, 1.0, "Run");// VISUALIZATION
    private final Button btnPlayPause = initializeButton(SVGIcon.playPause, 1.0, 1.0, "Play - Pause");// VISUALIZATION
    private final Button btnReplay = initializeButton(SVGIcon.replay, 1.0, 1.0, "Replay");// VISUALIZATION
    private final int MAX_NUM_ROWS = 200;// GRID
    private final int MAX_NUM_COLS = 400;// GRID
    private final int cellSize = 10;// GRID
    private final int numRows = MAX_NUM_ROWS;// GRID
    private final int numCols = MAX_NUM_COLS;// GRID
    private final int[][] environment = new int[numRows][numCols];// GRID
    private Label statusLabel; // Status shown in the bottom bar
    private Label statelabel; // Current editor mode
    private FileChooser fileChooser = new FileChooser();// VISUALIZATION
    private Stage primaryStage = null;
    // VISUALIZATION
    private ProgressIndicator progressIndicator;// VISUALIZATION
    private CanvasView canvas = null;
    private TextArea logArea;
    private String currentMapName = "Untitled (in-memory environment)";

    // Create button
    private Button initializeButton(String svgPathData, double scaleX, double scaleY, String tooltip) {
        SVGPath path = new SVGPath();
        path.setContent(svgPathData);
        path.setFill(Color.valueOf("#393E46"));
        path.setStroke(null);
        //path.setStroke(Color.valueOf("#393E46"));
        //path.setStrokeWidth(0.8);
        path.setStrokeLineCap(javafx.scene.shape.StrokeLineCap.ROUND);
        path.setStrokeLineJoin(javafx.scene.shape.StrokeLineJoin.ROUND);
        path.setScaleX(scaleX);
        path.setScaleY(scaleY);
        Button btn = new Button();
        btn.setGraphic(path);
        btn.setTooltip(new Tooltip(tooltip));
        btn.setStyle("-fx-background-color: transparent;");
        btn.setOnMouseEntered(e -> path.setFill(Color.valueOf("#0096FF")));
        btn.setOnMouseExited(e -> path.setFill(Color.valueOf("#393E46")));
        return btn;
    }

    // Create ToggleButton
    private ToggleButton initializeToggleButton(String svgPathData, double scaleX, double scaleY, String tooltip) {
        SVGPath path = new SVGPath();
        path.setContent(svgPathData);
        path.setFill(Color.valueOf("#393E46"));
        path.setStroke(null);
        //path.setStroke(Color.valueOf("#393E46"));
        //path.setStrokeWidth(0.8);
        path.setStrokeLineCap(javafx.scene.shape.StrokeLineCap.ROUND);
        path.setStrokeLineJoin(javafx.scene.shape.StrokeLineJoin.ROUND);
        path.setScaleX(scaleX);
        path.setScaleY(scaleY);

        ToggleButton btn = new ToggleButton();
        btn.setGraphic(path);
        btn.setTooltip(new Tooltip(tooltip));
        btn.setStyle("-fx-background-color: transparent;");

        // Hover
        btn.setOnMouseEntered(e -> {
            if (!btn.isSelected()) {
                path.setFill(Color.valueOf("#393E46"));
                path.setStroke(null);
            }
        });

        btn.setOnMouseExited(e -> {
            if (!btn.isSelected()) {
                path.setFill(Color.valueOf("#393E46"));
                path.setStroke(null);
            }
        });

        // Selected state
        btn.selectedProperty().addListener((obs, oldVal, isSelected) -> {
            if (isSelected) {
                path.setFill(Color.valueOf("#0096FF")); // warna aktif
            } else {
                path.setFill(Color.valueOf("#393E46")); // kembali normal
            }
        });

        return btn;
    }

    private void initializeToggleGroup() {
        btnTransform.setToggleGroup(toolGroup);
        btnAddDrone.setToggleGroup(toolGroup);
        btnRemoveDrone.setToggleGroup(toolGroup);
        btnWall.setToggleGroup(toolGroup);
        btnDrawLine.setToggleGroup(toolGroup);
        btnRemoveLine.setToggleGroup(toolGroup);

        // default tool
        btnTransform.setSelected(true);

        // aksi untuk tombol di toggle group
        toolGroup.selectedToggleProperty().addListener((obs, oldToggle, newToggle) -> {
            if (newToggle == null && oldToggle != null) {
                oldToggle.setSelected(true);
                return;
            }

            if (newToggle == btnTransform) {
                canvas.setFreeTransform();
            } else if (newToggle == btnAddDrone) {
                canvas.setState(State.ADD_DRONE);
            } else if (newToggle == btnRemoveDrone) {
                canvas.setState(State.REMOVE_DRONE);
            } else if (newToggle == btnWall) {
                canvas.setState(State.OBSTACLE);
            } else if (newToggle == btnDrawLine) {
                canvas.setState(State.LINE);
            } else if (newToggle == btnRemoveLine) {
                canvas.setState(State.REMOVE_LINE);
            }

            checkState();
        });
    }

    private CanvasView getCanvas() {
        if (canvas == null) {
            canvas = new CanvasView(environment, cellSize);
            canvas.setOnMousePressed(this::handleMousePressed);
            canvas.setOnMouseDragged(this::handleMouseDragged);
            canvas.setOnScroll(this::handleMouseScroll);
            canvas.setOnMouseClicked(this::handleMouseClicked);
            canvas.setOnMouseReleased(this::handleMouseReleased);
            canvas.setOnMouseMoved(this::handleMouseMoved);
        }
        return canvas;
    }

    private void handleMouseMoved(MouseEvent mouseEvent) {
        canvas.handleMouseMoved(mouseEvent);
    }

    private void handleMouseReleased(MouseEvent mouseEvent) {
        canvas.handleMouseReleased(mouseEvent);
    }

    private void handleMouseClicked(MouseEvent mouseEvent) {
        canvas.handleMouseClicked(mouseEvent);

    }

    private void handleMouseScroll(ScrollEvent scrollEvent) {
        canvas.handleMouseScroll(scrollEvent);
    }

    private void handleMouseDragged(MouseEvent mouseEvent) {
        canvas.handleMouseDragged(mouseEvent);
    }

    private void handleMousePressed(MouseEvent mouseEvent) {
        canvas.handleMousePressed(mouseEvent);
    }

    private void handleButtonOpen(ActionEvent actionEvent) {
        fileChooser = new FileChooser();
        fileChooser.setTitle("Open Environment");

        // Set initial directory "Documents"
        File documentsDir = new File(System.getProperty("user.home"), "Documents");
        if (documentsDir.exists()) {
            fileChooser.setInitialDirectory(documentsDir);
        }

        // Opsional: filter jenis file
        fileChooser.getExtensionFilters().addAll(
                new FileChooser.ExtensionFilter("Data", "*.sc"),
                new FileChooser.ExtensionFilter("All Files", "*.*")
        );

        File selectedFile = fileChooser.showOpenDialog(primaryStage);
        if (selectedFile != null) {
            // System.out.println("Selected File: " + selectedFile.getAbsolutePath());
            try {
                canvas.readFromFile(selectedFile);
                currentMapName = selectedFile.getName();
                clearSimulationLog();
                logArea.appendText("Environment loaded: " + currentMapName + "\n");
                statusLabel.setText("Opened: " + selectedFile.getName());
                checkState();
            } catch (RuntimeException ex) {
                showFileError("Open failed", ex.getMessage());
            }
        } else {
            // System.out.println("Tidak ada file dipilih.");
        }
    }


    private void handleButtonSave(ActionEvent actionEvent) {
        fileChooser = new FileChooser();
        fileChooser.setTitle("Save Environment");

        // Set initial directory "Documents"
        File documentsDir = new File(System.getProperty("user.home"), "Documents");
        if (documentsDir.exists()) {
            fileChooser.setInitialDirectory(documentsDir);
        }

        // Opsional: filter jenis file
        fileChooser.getExtensionFilters().addAll(
                new FileChooser.ExtensionFilter("Data", "*.sc"),
                new FileChooser.ExtensionFilter("All Files", "*.*")
        );

        File selectedFile = fileChooser.showSaveDialog(primaryStage);
        if (selectedFile != null) {
            // System.out.println("Selected File: " + selectedFile.getAbsolutePath());
            try {
                if (!selectedFile.getName().toLowerCase().endsWith(".sc"))
                    selectedFile = new File(selectedFile.getParentFile(), selectedFile.getName() + ".sc");
                canvas.writeToFile(selectedFile);
                currentMapName = selectedFile.getName();
                statusLabel.setText("Saved: " + selectedFile.getName());
                checkState();
            } catch (RuntimeException ex) {
                showFileError("Save failed", ex.getMessage());
            }
        } else {
            // System.out.println("Tidak ada file dipilih.");
        }
    }

    private void handleButtonRun(ActionEvent actionEvent) {
        canvas.setState(State.DEFAULT);
        checkState();
        clearSimulationLog();
        logArea.appendText("Calculating exploration...\n");
        try {
            canvas.run();
            logArea.setText(buildSimulationReport());
            logArea.positionCaret(0);
            statusLabel.setText("Exploration calculated — replay playing");
        } catch (RuntimeException ex) {
            logArea.appendText("SIMULATION ERROR: " + ex.getMessage() + "\n");
            showFileError("Simulation failed", ex.getMessage());
        }
    }

    private void handleButtonPlayPause(ActionEvent actionEvent) {
        canvas.setState(State.DEFAULT);
        checkState();
        if (!canvas.hasReplay()) {
            statusLabel.setText("Run the simulation first.");
            return;
        }
        canvas.togglePlayPause();
        statusLabel.setText(canvas.isPlaybackPlaying() ? "Playing" : "Paused");
    }

    private void handleButtonReplay(ActionEvent actionEvent) {
        canvas.setState(State.DEFAULT);
        checkState();
        if (!canvas.hasReplay()) {
            statusLabel.setText("Run the simulation first.");
            return;
        }
        canvas.replay();
        statusLabel.setText("Replay from timestep 0");
    }

    private void showFileError(String title, String message) {
        Alert alert = new Alert(Alert.AlertType.ERROR);
        alert.setTitle(title);
        alert.setHeaderText(title);
        alert.setContentText(message);
        alert.showAndWait();
        statusLabel.setText(title);
    }

    private void handleButtonResetEnvironment(ActionEvent actionEvent) {
        canvas.resetEnvironment();
        currentMapName = "Untitled (in-memory environment)";
        clearSimulationLog();
        logArea.appendText("Environment reset. Ready for a new simulation.\n");
        checkState();
    }

    private void clearSimulationLog() {
        if (logArea != null) logArea.clear();
    }

    // Presentation-only report: reads existing getters and the stored replay paths.
    private String buildSimulationReport() {
        DecentralizedExploration result = canvas.getLastExploration();
        if (result == null) return "No simulation result available.\n";

        StringBuilder sb = new StringBuilder();
        sb.append("DECENTRALIZED SWARM COVERAGE - EXPERIMENT LOG\n");
        sb.append("Algorithm \t: Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy\n");
        sb.append("Map file  \t\t: ").append(currentMapName).append('\n');
        sb.append(String.format(Locale.ROOT, "Grid size \t\t: %d x %d cells%n",
                canvas.getGridRows(), canvas.getGridCols()));
        sb.append("Drones    \t\t: ").append(canvas.getReplayDrones().size()).append('\n');
        sb.append("Sensing   \t\t: Ray-Casting 360 degrees; range ")
                .append(canvas.getSensorRangeCells()).append(" cells\n");
        sb.append("Planner   \t\t: Single-Source Dijkstra (8-connected)\n");
        sb.append("Frontier  \t\t: Local Occupancy Map + Frontier-Based Exploration\n");
        sb.append("Pheromone \t: Global shared; movement-based deposit; evaporation\n");
        sb.append(String.format(Locale.ROOT,
                "Parameters\t: lambda=%s, rho=%s, deposit=%s, recovery=%s, maxSteps=%s, stallLimit=%s%n",
                System.getProperty("swarm.lambda", "1.0"),
                System.getProperty("swarm.rho", "0.05"),
                System.getProperty("swarm.deposit", "1.0"),
                System.getProperty("swarm.recovery", "true"),
                System.getProperty("swarm.maxSteps", "2500"),
                System.getProperty("swarm.stallLimit", "30")));
        sb.append("\nSIMULATION RESULT\n");
        sb.append("Termination\t: ").append(result.getTerminationReason()).append('\n');
        boolean complete = "ALL_REACHABLE_FREE_CELLS_OBSERVED".equals(result.getTerminationReason());
        sb.append("Status\t\t: ").append(complete ? "COMPLETE" : "INCOMPLETE").append('\n');
        sb.append("\nFOUR PRIMARY EVALUATION METRICS\n");
        sb.append(String.format(Locale.ROOT, "1. Observed Coverage \t: %.2f%%%n", result.getObservedRatio()));
        sb.append(String.format(Locale.ROOT, "2. Revisit Ratio     \t\t: %.2f%%%n", result.getRevisit()));
        sb.append(String.format(Locale.ROOT, "3. Overlap Ratio     \t\t: %.2f%%%n", result.getOverlap()));
        sb.append("4. Completion Steps  \t: ").append(complete ? Integer.toString(result.getSteps()) : "N/A (incomplete run)").append('\n');
        if (!complete) sb.append("Simulation Steps     : ").append(result.getSteps()).append(" (termination before completion)\n");

        // EXPLORATION: reporting only. Reuse the algorithm's existing observed/reachable
        // masks and replay trajectories; no sensing, planning, or pheromone is modified.
        boolean[][] mustExplore = result.getMustExploreMask();
        int[][] observedAt = result.getFirstObservedAt();
        int mustExploredCells = 0;
        int exploredCells = 0;
        int minRow = Integer.MAX_VALUE, minCol = Integer.MAX_VALUE;
        int maxRow = -1, maxCol = -1;
        for (int r = 0; r < mustExplore.length; r++) {
            for (int c = 0; c < mustExplore[r].length; c++) {
                if (!mustExplore[r][c]) continue;
                mustExploredCells++;
                if (observedAt[r][c] >= 0) exploredCells++;
                minRow = Math.min(minRow, r);
                minCol = Math.min(minCol, c);
                maxRow = Math.max(maxRow, r);
                maxCol = Math.max(maxCol, c);
            }
        }

        // Exactly mirrors visit() semantics: start cells count as visits;
        // stationary replay frames do NOT count as additional visits.
        int gridCols = canvas.getGridCols();
        java.util.Map<Integer, Integer> cellVisits = new java.util.HashMap<>();
        java.util.Map<Integer, java.util.Set<Integer>> cellDroneIds = new java.util.HashMap<>();
        java.util.List<Drone> replayDrones = canvas.getReplayDrones();
        for (int i = 0; i < replayDrones.size(); i++) {
            Point previous = null;
            for (Point point : replayDrones.get(i).path) {
                if (point.equals(previous)) continue;
                int key = point.row() * gridCols + point.col();
                cellVisits.merge(key, 1, Integer::sum);
                cellDroneIds.computeIfAbsent(key, ignored -> new java.util.HashSet<>()).add(i);
                previous = point;
            }
        }
        long numRevisitCells = cellVisits.values().stream().filter(count -> count > 1).count();
        long numOverlapCells = cellDroneIds.values().stream().filter(ids -> ids.size() > 1).count();

        sb.append("\nEXPLORATION\n");
        sb.append("1. Must Explored Cells \t: ").append(mustExploredCells).append('\n');
        sb.append("2. Explored Cells      \t: ").append(exploredCells).append('\n');
        sb.append("3. MIN Row             \t\t: ").append(mustExploredCells == 0 ? "N/A" : minRow).append('\n');
        sb.append("4. MIN Col             \t\t: ").append(mustExploredCells == 0 ? "N/A" : minCol).append('\n');
        sb.append("5. MAX Row             \t: ").append(mustExploredCells == 0 ? "N/A" : maxRow).append('\n');
        sb.append("6. MAX Col             \t\t: ").append(mustExploredCells == 0 ? "N/A" : maxCol).append('\n');
        sb.append("7. Observation Area    \t: ").append(Math.abs(maxRow-minRow)+" X "+Math.abs(maxCol-minCol)).append('\n');
        sb.append("8. Num Revisit Cells   \t: ").append(numRevisitCells).append('\n');
        sb.append("9. Num Overlap Cells   \t: ").append(numOverlapCells).append('\n');

        sb.append("\nDRONE TRAJECTORIES (row,col; zero-based)\n");
        sb.append("Includes start point and stationary frames. Each point corresponds to one replay frame.\n");
        for (int i = 0; i < replayDrones.size(); i++) {
            Drone d = replayDrones.get(i);
            sb.append("D").append(i + 1).append("\t: ");
            if (d.path == null || d.path.isEmpty()) {
                sb.append("(no trajectory)");
            } else {
                for (int j = 0; j < d.path.size(); j++) {
                    Point point = d.path.get(j);
                    if (j != 0) sb.append('-');
                    sb.append('(').append(point.row()).append(',').append(point.col()).append(')');
                }
            }
            sb.append('\n');
        }
        return sb.toString();
    }

    private void checkState() {
        if (canvas.getState().equals(State.DEFAULT)) {
            statelabel.setText("STATE: " + "FREE TRANSFORM" + "\t-\t");
            btnTransform.setSelected(true);
        } else if (canvas.getState().equals(State.ADD_DRONE)) {
            statelabel.setText("STATE: " + "ADD DRONE" + "\t-\t");
            btnAddDrone.setSelected(true);
        } else if (canvas.getState().equals(State.REMOVE_DRONE)) {
            statelabel.setText("STATE: " + "REMOVE DRONE" + "\t-\t");
            btnRemoveDrone.setSelected(true);
        } else if (canvas.getState().equals(State.OBSTACLE)) {
            statelabel.setText("STATE: " + "DRAW OBSTACLE" + "\t-\t");
            btnWall.setSelected(true);
        } else if (canvas.getState().equals(State.LINE)) {
            statelabel.setText("STATE: " + "DRAW LINE" + "\t-\t");
            btnDrawLine.setSelected(true);
        } else if (canvas.getState().equals(State.REMOVE_LINE)) {
            statelabel.setText("STATE: " + "REMOVE LINE" + "\t-\t");
            btnRemoveLine.setSelected(true);
        }
    }

    @Override
    public void start(Stage stage) throws Exception {
        primaryStage = stage;

        // === Sidebar ===
        VBox sidebar = new VBox(2);
        sidebar.getStyleClass().add("sidebar");
        sidebar.setPadding(new Insets(4));

        // button
        initializeToggleGroup();
        // set on action for buttons
        btnOpen.setOnAction(this::handleButtonOpen);
        btnSave.setOnAction(this::handleButtonSave);
        btnRun.setOnAction(this::handleButtonRun);
        btnPlayPause.setOnAction(this::handleButtonPlayPause);
        btnReplay.setOnAction(this::handleButtonReplay);
        btnNew.setOnAction(this::handleButtonResetEnvironment);
        VBox topButtons = new VBox(2, btnNew, btnOpen, btnTransform, btnAddDrone, btnRemoveDrone, btnWall, btnDrawLine, btnRemoveLine, btnSave, btnRun, btnPlayPause, btnReplay);
        topButtons.setAlignment(Pos.TOP_CENTER);

        progressIndicator = new ProgressIndicator(0);
        progressIndicator.setVisible(false);

        VBox bottomButtons = new VBox(2, progressIndicator);
        bottomButtons.setAlignment(Pos.BOTTOM_CENTER);
        VBox.setVgrow(topButtons, Priority.ALWAYS);
        sidebar.getChildren().addAll(topButtons, bottomButtons);

        // === Center Pane ===
        BorderPane centerPane = new BorderPane();
        centerPane.getStyleClass().add("center-pane");
        StackPane canvasPane = new StackPane();

        getCanvas();
        canvas.render();
        canvasPane.getChildren().add(canvas);

        // Scroll Pane
        ScrollPane scrollPane = new ScrollPane(canvasPane);
        scrollPane.setFitToWidth(true);
        scrollPane.setFitToHeight(true);
        scrollPane.setHbarPolicy(ScrollPane.ScrollBarPolicy.ALWAYS);
        scrollPane.setVbarPolicy(ScrollPane.ScrollBarPolicy.ALWAYS);
        scrollPane.setPrefHeight(900);

        // === Log Area ===

        // === Log Area ===
        logArea = new TextArea();
        logArea.setPrefRowCount(3);
        logArea.setWrapText(false);
        logArea.setEditable(false);
        logArea.setBorder(null);
        logArea.setText("Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy\n");
        MenuItem clearLogItem = new MenuItem("Clear Log");
        clearLogItem.setOnAction(e -> clearSimulationLog());
        MenuItem copyLogItem = new MenuItem("Copy All");
        copyLogItem.setOnAction(e -> {
            javafx.scene.input.ClipboardContent content = new javafx.scene.input.ClipboardContent();
            content.putString(logArea.getText());
            javafx.scene.input.Clipboard.getSystemClipboard().setContent(content);
        });
        logArea.setContextMenu(new ContextMenu(clearLogItem, copyLogItem));


        // === Status Bar ===
        statusLabel = new Label("Ready.");
        statusLabel.getStyleClass().add("status-label");
        statelabel = new Label("STATE: " + canvas.getState().toString() + "\t-\t");
        statelabel.getStyleClass().add("status-label");
        HBox statusBar = new HBox(statelabel, statusLabel);
        statusBar.getStyleClass().add("status-bar");
        statusBar.setPadding(new Insets(5));

        // SplitPane vertikal
        SplitPane splitPane = new SplitPane(scrollPane, logArea);
        splitPane.setOrientation(Orientation.VERTICAL);
        splitPane.setDividerPositions(0.90);

        // === Bottom (log + status) ===
        VBox bottomBox = new VBox(statusBar);

        centerPane.setCenter(splitPane);
        centerPane.setBottom(bottomBox);

        // === Layout ===
        BorderPane root = new BorderPane();
        //root.setTop(topBar);
        root.setLeft(sidebar);
        root.setCenter(centerPane);
        Scene scene = new Scene(root, 1000, 600);
        scene.getStylesheets().add(CSS.style);

        /* STRART the ENGINE */
        AnimationTimer engine = new AnimationTimer() {
            @Override
            public void handle(long l) {
                // UPDATE
                canvas.update();

                // RENDER
                canvas.render();
            }
        };
        engine.start();

        // stage
        primaryStage.setTitle("Robotics & Mechatronics Laboratory - Mechanical Engineering - Hasanuddin University");
        primaryStage.setMaximized(true);
        primaryStage.setScene(scene);
        primaryStage.show();
        primaryStage.setOnCloseRequest(new EventHandler<WindowEvent>() {
            @Override
            public void handle(WindowEvent windowEvent) {
                Platform.exit();
                System.exit(0);
            }
        });
        primaryStage.getIcons().add(new Image(this.getClass().getResourceAsStream("icon.png")));
    }
}