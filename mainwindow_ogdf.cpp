#include "mainwindow.h"
#include <QApplication>
#include <QDebug>
#include <QPainterPath>
#include <QMouseEvent>
#include <QGraphicsRectItem>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , circuitGraph(new CircuitGraph())
{
    setWindowTitle("Verilog Schematic Viewer");
    resize(1200, 800);
    
    // UI Setup
    scene = new QGraphicsScene(this);
    scene->setBackgroundBrush(Qt::white);
    
    view = new QGraphicsView(scene, this);
    view->setRenderHint(QPainter::Antialiasing);
    view->setDragMode(QGraphicsView::ScrollHandDrag); // Enable panning
    setCentralWidget(view);
    
    // Toolbar
    QToolBar *toolbar = addToolBar("Main");
    
    QAction *openAct = new QAction("Open Verilog...", this);
    connect(openAct, &QAction::triggered, this, &MainWindow::openFile);
    toolbar->addAction(openAct);
    
    toolbar->addSeparator();
    
    toolbar->addWidget(new QLabel(" Module: "));
    moduleCombo = new QComboBox();
    moduleCombo->setMinimumWidth(150);
    connect(moduleCombo, QOverload<const QString&>::of(&QComboBox::currentTextChanged),
            this, &MainWindow::onModuleChanged);
    toolbar->addWidget(moduleCombo);
    
    toolbar->addSeparator();
    
    QAction *zoomInAct = new QAction("Zoom In", this);
    connect(zoomInAct, &QAction::triggered, this, &MainWindow::zoomIn);
    toolbar->addAction(zoomInAct);
    
    QAction *zoomOutAct = new QAction("Zoom Out", this);
    connect(zoomOutAct, &QAction::triggered, this, &MainWindow::zoomOut);
    toolbar->addAction(zoomOutAct);
    
    QAction *resetAct = new QAction("Reset", this);
    connect(resetAct, &QAction::triggered, this, &MainWindow::resetZoom);
    toolbar->addAction(resetAct);

    toolbar->addSeparator();

    backAction = new QAction("Build Hierarchy Back", this);
    connect(backAction, &QAction::triggered, this, &MainWindow::navigateBack);
    backAction->setEnabled(false);
    toolbar->addAction(backAction);
    
    // Try auto-load
    QFileInfo fi("test_circuit.v");
    if(fi.exists()) loadNetlist(fi.absoluteFilePath());
}

MainWindow::~MainWindow() {
    delete circuitGraph;
}

void MainWindow::openFile() {
    QString fileName = QFileDialog::getOpenFileName(this, "Open Verilog", "", "Verilog (*.v);;All (*)");
    if (!fileName.isEmpty()) loadNetlist(fileName);
}

void MainWindow::loadNetlist(const QString &filename) {
    currentNetlist = NetlistParser::parse(filename.toStdString());
    
    moduleCombo->blockSignals(true);
    moduleCombo->clear();
    for (const auto& pair : currentNetlist.modules) {
        moduleCombo->addItem(QString::fromStdString(pair.first));
    }
    
    if (!currentNetlist.topModule.empty()) {
        moduleCombo->setCurrentText(QString::fromStdString(currentNetlist.topModule));
        currentModuleName = currentNetlist.topModule;
    } else if (!currentNetlist.modules.empty()) {
        moduleCombo->setCurrentIndex(0);
        currentModuleName = moduleCombo->currentText().toStdString();
    }
    moduleCombo->blockSignals(false);
    
    drawModule();
}

void MainWindow::onModuleChanged(const QString& moduleName) {
    currentModuleName = moduleName.toStdString();
    drawModule();
}

void MainWindow::drawModule() {
    clearScene();
    if (currentModuleName.empty()) return;
    
    circuitGraph->buildFromModule(currentNetlist, currentModuleName);
    circuitGraph->applyLayout();
    
    drawEdges();

    // Draw Graph
    // 0. Draw expanded clusters
    const ogdf::ClusterGraph& cg = circuitGraph->clusterG;
    const ogdf::ClusterGraphAttributes* cga = circuitGraph->cga;
    
    // Simple recursive lambda to draw cluster boxes
    std::function<void(ogdf::cluster)> drawCluster = [&](ogdf::cluster c) {
        if (!c) return;
        
        // Draw children first? Or self first?
        // Draw self first as background.
        double x = cga->x(c);
        double y = cga->y(c);
        double w = cga->width(c);
        double h = cga->height(c);
        
        // Rotate 90 deg
        double drawX = y;
        double drawY = x;
        double drawW = h;
        double drawH = w;
        
        QGraphicsRectItem* rect = scene->addRect(drawX - drawW/2, drawY - drawH/2, drawW, drawH);
        rect->setBrush(QColor(255, 253, 220)); // Light yellow for expanded groups
        rect->setPen(QPen(Qt::darkGray, 1, Qt::DashLine));
        rect->setZValue(-1); // Behind nodes
        
        // Label
        QGraphicsTextItem* text = scene->addText(QString::fromStdString(cga->label(c)));
        text->setPos(drawX - drawW/2 + 5, drawY - drawH/2 + 5);
        text->setZValue(-0.5);
        
        // Recurse
        for(auto it = c->cBegin(); it.valid(); it = it.succ()) {
            drawCluster(*it);
        }
    };
    
    if (cg.rootCluster()) {
        for(auto it = cg.rootCluster()->cBegin(); it.valid(); it = it.succ()) {
            drawCluster(*it);
        }
    }
    
    // Iterate OGDF nodes
    for(ogdf::node n = circuitGraph->g.firstNode(); n; n = n->succ()) {
        double x = circuitGraph->ga->x(n);
        double y = circuitGraph->ga->y(n);
        double w = circuitGraph->ga->width(n);
        double h = circuitGraph->ga->height(n);
        
        // Sugiyama is Top-Down by default
        // Simple 90-degree rotation: (X, Y) -> (Y, X)
        double drawX = y;
        double drawY = x;
        double drawW = h;
        double drawH = w;
        
        if (circuitGraph->nodeDataMap.count(n)) {
            const auto& data = circuitGraph->nodeDataMap.at(n);
            if (data.type == CircuitNodeType::ModuleInstance) {
                drawInstance(data, drawX, drawY, drawW, drawH);
            } else if (data.type == CircuitNodeType::Port) {
                drawPort(data, drawX, drawY, drawW, drawH);
            } else if (data.type == CircuitNodeType::NetJoint) {
                drawNetJoint(data, drawX, drawY, drawW, drawH);
            }
        }
    }
    
    scene->update();
}


void MainWindow::drawInstance(const GraphNodeData& data, double x, double y, double w, double h) {
    // x, y is center
    QGraphicsRectItem* rect = scene->addRect(x - w/2, y - h/2, w, h);
    rect->setBrush(QColor(230, 240, 255));
    // If instance is clickable (it's a module), color it slightly differently or change cursor?
    // Store module name in item data (UserRole)
    if (currentNetlist.modules.count(data.moduleType)) {
        rect->setBrush(QColor(200, 220, 255)); // Lighter blue for modules
        rect->setPen(QPen(Qt::darkBlue, 2));
        rect->setData(0, QString::fromStdString(data.moduleType));
        rect->setData(1, QString::fromStdString(data.path)); // Store path for expansion
        rect->setCursor(Qt::PointingHandCursor);
    } else {
        rect->setBrush(QColor(240, 240, 240)); // Grey for simple cells
        rect->setPen(QPen(Qt::black, 1));
        rect->setData(0, QString::fromStdString(data.moduleType));
    }

    // Draw Pins
    if (currentNetlist.modules.count(data.moduleType)) {
        const Module& mod = currentNetlist.modules.at(data.moduleType);
        
        // Count inputs and outputs
        std::vector<std::string> inputs, outputs;
        for (const auto& p : mod.ports) {
            if (p.direction == "input") inputs.push_back(p.name);
            else outputs.push_back(p.name);
        }
        
        // Draw Inputs (Left)
        double stepY = h / (inputs.size() + 1);
        for (size_t i = 0; i < inputs.size(); ++i) {
            double py = (y - h/2) + stepY * (i + 1);
            double px = x - w/2;
            
            QGraphicsRectItem* pin = scene->addRect(px - 4, py - 4, 8, 8);
            pin->setBrush(QColor(200, 200, 200));
            pin->setPen(QPen(Qt::black, 1));
            
             QGraphicsTextItem* t = scene->addText(QString::fromStdString(inputs[i]));
             t->setScale(0.6);
             t->setPos(px + 2, py - 10);
        }
        
        // Draw Outputs (Right)
        stepY = h / (outputs.size() + 1);
        for (size_t i = 0; i < outputs.size(); ++i) {
            double py = (y - h/2) + stepY * (i + 1);
            double px = x + w/2;
            
            QGraphicsRectItem* pin = scene->addRect(px - 4, py - 4, 8, 8);
            pin->setBrush(QColor(200, 200, 200));
            pin->setPen(QPen(Qt::black, 1));

             QGraphicsTextItem* t = scene->addText(QString::fromStdString(outputs[i]));
             t->setScale(0.6);
             t->setPos(px - 15, py - 10);
        }
    }
    
    QGraphicsTextItem* text = scene->addText(QString::fromStdString(data.label));
    QRectF b = text->boundingRect();
    text->setPos(x - b.width()/2, y - b.height()/2);
}

void MainWindow::drawPort(const GraphNodeData& data, double x, double y, double w, double h) {
    QAbstractGraphicsShapeItem* shape;
    if (data.direction == "input") {
         shape = scene->addRect(x - w/2, y - h/2, w, h);
         shape->setBrush(QColor(200, 255, 200));
    } else {
         shape = scene->addRect(x - w/2, y - h/2, w, h);
         shape->setBrush(QColor(255, 200, 200));
    }
    shape->setPen(QPen(Qt::black));

    QGraphicsTextItem* text = scene->addText(QString::fromStdString(data.label));
    QRectF b = text->boundingRect();
    text->setPos(x - b.width()/2, y - b.height()/2);
}

void MainWindow::drawNetJoint(const GraphNodeData& data, double x, double y, double w, double h) {
    scene->addEllipse(x - w/2, y - h/2, w, h, QPen(Qt::black), QBrush(Qt::black));
}

// Helper to get pin coordinates
QPointF MainWindow::getPinCoordinates(ogdf::node n, const std::string& pinName) {
    // Current node geometry (rotated)
    double cx = circuitGraph->ga->y(n);  // Swapped coordinates for display rotation
    double cy = circuitGraph->ga->x(n);
    double w = circuitGraph->ga->height(n);
    double h = circuitGraph->ga->width(n);

    if (!circuitGraph->nodeDataMap.count(n)) return QPointF(cx, cy);
    const auto& data = circuitGraph->nodeDataMap.at(n);
    
    // Check if it's the Port node of the PARENT module (the container)
    // If n is a Port node:
    //   Input Port -> Usually sits on Left side of schematic, connections go to its Right side.
    //   Output Port -> Sits on Right side, connections come from its Left side.
    if (data.type == CircuitNodeType::Port) {
        if (data.direction == "input") return QPointF(cx + w/2, cy); // Connect to right side
        return QPointF(cx - w/2, cy); // Connect to left side
    }
    
    if (data.type != CircuitNodeType::ModuleInstance) return QPointF(cx, cy);
    
    // Look up module definition
    if (currentNetlist.modules.count(data.moduleType) == 0) return QPointF(cx, cy);
    const Module& mod = currentNetlist.modules.at(data.moduleType);
    
    // Find pin in ports
    int idx = -1;
    int count = 0;
    bool isInput = false;
    
    // First, identify if this pin name is in input list or output list
    std::vector<std::string> inputs, outputs;
    for (const auto& p : mod.ports) {
        if (p.direction == "input") inputs.push_back(p.name);
        else outputs.push_back(p.name);
    }
    
    // Check inputs
    for (int i=0; i<inputs.size(); ++i) {
        if (inputs[i] == pinName) {
            idx = i;
            count = inputs.size();
            isInput = true;
            break;
        }
    }
    // Check outputs
    if (!isInput) {
        for (int i=0; i<outputs.size(); ++i) {
             if (outputs[i] == pinName) {
                idx = i;
                count = outputs.size();
                isInput = false;
                break;
            }
        }
    }
    
    if (idx == -1) return QPointF(cx, cy); // Not found
    
    double stepY = h / (count + 1);
    double py = (cy - h/2) + stepY * (idx + 1);
    
    if (isInput) return QPointF(cx - w/2, py); // Left edge
    else return QPointF(cx + w/2, py);         // Right edge
}

void MainWindow::drawEdges() {
    for(ogdf::edge e = circuitGraph->g.firstEdge(); e; e = e->succ()) {
        QPainterPath path;
        
        ogdf::node src = e->source();
        ogdf::node tgt = e->target();
        
        // Retrieve Pin Info
        std::string srcPin = "";
        std::string tgtPin = "";
        if (circuitGraph->edgePinMap.count(e)) {
            srcPin = circuitGraph->edgePinMap[e].sourcePin;
            tgtPin = circuitGraph->edgePinMap[e].targetPin;
        }

        // Start point
        QPointF startPt;
        if (!srcPin.empty()) {
            startPt = getPinCoordinates(src, srcPin);
        } else {
            // Default center/edge
             double sx = circuitGraph->ga->y(src);
             double sy = circuitGraph->ga->x(src);
             startPt = QPointF(sx, sy);
        }
        
        // End point
        QPointF endPt;
        if (!tgtPin.empty()) {
            endPt = getPinCoordinates(tgt, tgtPin);
        } else {
             double tx = circuitGraph->ga->y(tgt);
             double ty = circuitGraph->ga->x(tgt);
             endPt = QPointF(tx, ty);
        }
        
        // Route
        QVector<QPointF> points;
        points.append(startPt);
        
        // Bends? 
        // If we use bends from Sugiyama, they are calculated based on Node Centers.
        // They might look weird if we shift start/end to pins.
        // Let's try to just use orthgonal routing between the two pin points directly, ignoring Sugiyama bends for now?
        // Or mix them?
        // If endpoints are close, direct routing is better.
        // If far, we need bends to avoid obstacles.
        // PROPER WAY: Reroute using the pin coords.
        // SIMPLE WAY (for now): H-V-H between startPt and endPt.
        
        points.append(endPt);
        
        path.moveTo(points[0]);
        // Do orthogonal routing manually
        QPointF p1 = points[0];
        QPointF p2 = points[points.size()-1];
        
        double midX = (p1.x() + p2.x()) / 2.0;
        
        // Heuristic: If one is a "Joint", maybe just V-H or H-V?
        // Standard Schematic:
        // Output -> Horizontal -> Vertical -> Horizontal -> Input
        
        path.lineTo(midX, p1.y());
        path.lineTo(midX, p2.y());
        path.lineTo(p2);
        
        scene->addPath(path, QPen(Qt::black, 1.5));
    }
}


void MainWindow::mouseDoubleClickEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        // Convert window pos to view pos
        QPoint viewPos = view->mapFrom(this, event->pos());
        QPointF scenePos = view->mapToScene(viewPos);
        
        QGraphicsItem *item = scene->itemAt(scenePos, view->transform());
        
        // Find topmost item that has data
        while(item) {
             QVariant data = item->data(0);
             if (!data.toString().isEmpty()) {
                 std::string mod = data.toString().toStdString();
                 // Get path data if set
                 QVariant pathVar = item->data(1);
                 
                 // If Shift held -> Expand In Place
                 if (event->modifiers() & Qt::ShiftModifier) {
                     if (!pathVar.toString().isEmpty()) {
                         expandInstance(pathVar.toString().toStdString());
                         return;
                     }
                 }
                 
                 // Else -> Default Navigate
                 if (currentNetlist.modules.count(mod)) {
                     // Navigate
                     std::string prev = currentModuleName;
                     navigationHistory.push_back(prev);
                     backAction->setEnabled(true);
                     
                     moduleCombo->setCurrentText(QString::fromStdString(mod));
                     return;
                 }
             }
             item = item->parentItem();
        }
    }
    QMainWindow::mouseDoubleClickEvent(event);
}

void MainWindow::expandInstance(const std::string& fullInstName) {
    if (!circuitGraph->expandedInstances.count(fullInstName)) {
        circuitGraph->expandedInstances.insert(fullInstName);
    } else {
        circuitGraph->expandedInstances.erase(fullInstName); // Toggle off
    }
    drawModule();
}

void MainWindow::navigateToModule(const std::string& moduleName) {
    // Helper used by double click
    if (moduleName == currentModuleName) return;
    
    navigationHistory.push_back(currentModuleName);
    backAction->setEnabled(true);
    
    // Update combo
    moduleCombo->blockSignals(true);
    moduleCombo->setCurrentText(QString::fromStdString(moduleName));
    moduleCombo->blockSignals(false);
    
    // Draw
    onModuleChanged(QString::fromStdString(moduleName));
}

void MainWindow::navigateBack() {
    if (navigationHistory.empty()) return;
    
    std::string moduleName = navigationHistory.back();
    navigationHistory.pop_back();
    
    if (navigationHistory.empty()) backAction->setEnabled(false);
    
    moduleCombo->blockSignals(true);
    moduleCombo->setCurrentText(QString::fromStdString(moduleName));
    moduleCombo->blockSignals(false);
    
    currentModuleName = moduleName;
    drawModule();
}

void MainWindow::clearScene() {
    scene->clear();
    scene->setSceneRect(-2000, -2000, 4000, 4000); // Reset generic rect
}

void MainWindow::zoomIn() { view->scale(1.2, 1.2); }
void MainWindow::zoomOut() { view->scale(1.0/1.2, 1.0/1.2); }
void MainWindow::resetZoom() { view->resetTransform(); }
