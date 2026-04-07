#include "mainwindow.h"
#include <QApplication>
#include <QDebug>
#include <QPainterPath>
#include <QMouseEvent>
#include <QGraphicsRectItem>
#include <QGraphicsPolygonItem>
#include <functional>
#include <iostream>

// Helper to get absolute position
QPointF getAbsolutePos(CNode* n) {
    double x = n->x;
    double y = n->y;
    CNode* p = n->parent;
    while(p) {
        x += p->x;
        y += p->y;
        p = p->parent;
    }
    return QPointF(x, y);
}

// Helper to get pin coordinates on a node
QPointF getPinCoordinates(CNode* n, const std::string& pinName, const Netlist& netlist) {
    QPointF absPos = getAbsolutePos(n);

    // Use ELK layout coordinates if available
    for (const auto& p : n->ports) {
        if (p.name == pinName) {
            // ELK ports are usually center relative or top-left relative depending on config
            // But based on circuitgraph.h, we initialize them top-left relative.
            return QPointF(absPos.x() + p.x, absPos.y() + p.y);
        }
    }

    double cx = absPos.x() + n->width/2;
    double cy = absPos.y() + n->height/2;
    double w = n->width;
    double h = n->height;
    
    if (n->data.type == CircuitNodeType::Port) {
        // Port node itself (usually small box)
        if (n->data.direction == "input") return QPointF(cx + w/2, cy);
        return QPointF(cx - w/2, cy);
    }
    
    if (n->data.type != CircuitNodeType::ModuleInstance) return QPointF(cx, cy);
    if (netlist.modules.count(n->data.moduleType) == 0) return QPointF(cx, cy);
    const Module& mod = netlist.modules.at(n->data.moduleType);
    
    std::vector<std::string> inputs, outputs;
    for (const auto& p : mod.ports) {
        if (p.direction == "input") inputs.push_back(p.name);
        else outputs.push_back(p.name);
    }
    
    int idx = -1;
    bool isInput = false;
    double count = 0;
    
    for(size_t i=0; i<inputs.size(); ++i) if(inputs[i] == pinName) { idx=i; isInput=true; count=inputs.size(); break; }
    if(idx == -1) for(size_t i=0; i<outputs.size(); ++i) if(outputs[i] == pinName) { idx=i; isInput=false; count=outputs.size(); break; }
    
    if (idx == -1) return QPointF(cx, cy);
    
    double stepY = h / (count + 1);
    double py = (absPos.y()) + stepY * (idx + 1);
    
    if (isInput) return QPointF(absPos.x(), py);
    return QPointF(absPos.x() + w, py);
}


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , circuitGraph(new CircuitGraph())
{
    setWindowTitle("Verilog Schematic Viewer (ELK Layout)");
    resize(1200, 800);
    
    scene = new QGraphicsScene(this);
    scene->setBackgroundBrush(Qt::white);
    
    view = new QGraphicsView(scene, this);
    view->setRenderHint(QPainter::Antialiasing);
    view->setDragMode(QGraphicsView::ScrollHandDrag); 
    setCentralWidget(view);
    
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
    
    QAction *fitAct = new QAction("Fit", this);
    connect(fitAct, &QAction::triggered, this, &MainWindow::fitInView);
    toolbar->addAction(fitAct);
    
    toolbar->addSeparator();
    backAction = new QAction("Build Hierarchy Back", this);
    connect(backAction, &QAction::triggered, this, &MainWindow::navigateBack);
    backAction->setEnabled(false);
    toolbar->addAction(backAction);
    
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
    
    // Draw Nodes
    for(CNode* n : circuitGraph->nodes) {
        QPointF absPos = getAbsolutePos(n);
        double cx = absPos.x() + n->width/2;
        double cy = absPos.y() + n->height/2;
        
        const auto& data = n->data;
        if (data.type == CircuitNodeType::ModuleInstance) {
            drawInstance(n, cx, cy);
        } else if (data.type == CircuitNodeType::Port) {
            drawPort(data, cx, cy, n->width, n->height);
        } else if (data.type == CircuitNodeType::NetJoint) {
            drawNetJoint(data, cx, cy, n->width, n->height);
        }
    }
    
    drawEdges();
    scene->update();
}

void MainWindow::drawInstancePins(CNode* n, double cx, double cy) {
    const GraphNodeData& data = n->data;
    double w = n->width;
    double h = n->height;
    double x = cx;  // CENTER
    double y = cy;  // CENTER
    
    if (!n->ports.empty()) {
        for(const auto& p : n->ports) {
            double px = (cx - w/2) + p.x;
            double py = (cy - h/2) + p.y;
            scene->addRect(px, py, 4, 4, QPen(Qt::black), QBrush(Qt::gray));
            QGraphicsTextItem* t = scene->addText(QString::fromStdString(p.name));
            t->setScale(0.6);
            if (p.x < w / 2) t->setPos(px + 2, py - 8);
            else t->setPos(px - 15, py - 8);
        }
    } else if (data.type == CircuitNodeType::ModuleInstance && currentNetlist.modules.count(data.moduleType)) {
        // Fallback: use module ports from netlist
        const Module& mod = currentNetlist.modules.at(data.moduleType);
        std::vector<std::string> inputs, outputs;
        for (const auto& p : mod.ports) {
            if (p.direction == "input") inputs.push_back(p.name);
            else outputs.push_back(p.name);
        }
        
        double stepY = h / (inputs.size() + 1);
        for (size_t i = 0; i < inputs.size(); ++i) {
            double py = (y - h/2) + stepY * (i + 1);
            double px = x - w/2;
            scene->addRect(px - 2, py - 2, 4, 4, QPen(Qt::black), QBrush(Qt::gray));
            QGraphicsTextItem* t = scene->addText(QString::fromStdString(inputs[i]));
            t->setScale(0.6); t->setPos(px + 2, py - 8);
        }
        stepY = h / (outputs.size() + 1);
        for (size_t i = 0; i < outputs.size(); ++i) {
            double py = (y - h/2) + stepY * (i + 1);
            double px = x + w/2;
            scene->addRect(px - 2, py - 2, 4, 4, QPen(Qt::black), QBrush(Qt::gray));
            QGraphicsTextItem* t = scene->addText(QString::fromStdString(outputs[i]));
            t->setScale(0.6); t->setPos(px - 15, py - 8);
        }
    }
}

void MainWindow::drawInstance(CNode* n, double cx, double cy) {
    const GraphNodeData& data = n->data;
    double w = n->width;
    double h = n->height;
    double x = cx;
    double y = cy;

    // x, y is CENTER
    QGraphicsRectItem* rect = scene->addRect(x - w/2, y - h/2, w, h);
    
    if (data.type == CircuitNodeType::ModuleInstance && currentNetlist.modules.count(data.moduleType)) {
        rect->setBrush(QColor(200, 220, 255)); 
        rect->setPen(QPen(Qt::darkBlue, 2));
        rect->setData(0, QString::fromStdString(data.moduleType));
        rect->setData(1, QString::fromStdString(data.path)); 
        rect->setCursor(Qt::PointingHandCursor);
        
        // Draw Pins from n->ports or module
        drawInstancePins(n, cx, cy);
    } else if (data.type == CircuitNodeType::ModuleInstance) {
        // External module - still try to draw pins from n->ports
        rect->setBrush(QColor(240, 240, 240)); 
        rect->setPen(QPen(Qt::black, 1));
        if (!n->ports.empty()) {
            drawInstancePins(n, cx, cy);
        }
    } else {
        rect->setBrush(QColor(240, 240, 240)); 
        rect->setPen(QPen(Qt::black, 1));
    }
    
    QGraphicsTextItem* text = scene->addText(QString::fromStdString(data.label));
    QRectF b = text->boundingRect();
    text->setPos(x - b.width()/2, y - b.height()/2);
}

void MainWindow::drawPort(const GraphNodeData& data, double x, double y, double w, double h) {
    QAbstractGraphicsShapeItem* shape;
    if (data.direction == "input") {
         QPolygonF p; p << QPointF(x-w/2, y-h/2) << QPointF(x+w/2, y) << QPointF(x-w/2, y+h/2);
         shape = scene->addPolygon(p);
         shape->setBrush(QColor(200, 255, 200));
    } else {
         QPolygonF p; p << QPointF(x-w/2, y-h/2) << QPointF(x+w/2, y) << QPointF(x-w/2, y+h/2);
         shape = scene->addPolygon(p);
         shape->setBrush(QColor(255, 200, 200));
    }
    shape->setPen(QPen(Qt::black));
    QGraphicsTextItem* text = scene->addText(QString::fromStdString(data.label));
    text->setPos(x - 10, y - 20);
}

void MainWindow::drawNetJoint(const GraphNodeData& data, double x, double y, double w, double h) {
    scene->addEllipse(x - w/2, y - h/2, w, h, QPen(Qt::black), QBrush(Qt::black));
}

void MainWindow::drawEdges() {
    for(CEdge* e : circuitGraph->edges) {
        if (e->points.empty()) continue;
        
        QPainterPath path;
        path.moveTo(e->points[0]);
        for(size_t i = 1; i < e->points.size(); ++i) {
            path.lineTo(e->points[i]);
        }
        
        scene->addPath(path, QPen(Qt::black, 1));
    }
}

void MainWindow::zoomIn() { view->scale(1.2, 1.2); }
void MainWindow::zoomOut() { view->scale(1/1.2, 1/1.2); }
void MainWindow::resetZoom() { view->resetTransform(); }
void MainWindow::fitInView() {
    view->resetTransform();
    view->fitInView(scene->itemsBoundingRect(), Qt::KeepAspectRatio);
    view->scale(0.9, 0.9); // Slight margin
}

void MainWindow::mouseDoubleClickEvent(QMouseEvent *event) {
    QMainWindow::mouseDoubleClickEvent(event);
}

void MainWindow::wheelEvent(QWheelEvent *event) {
    // Only zoom when Ctrl is pressed, otherwise allow default pan behavior
    if (event->modifiers() & Qt::ControlModifier) {
        if (event->angleDelta().y() > 0) {
            view->scale(1.1, 1.1);
        } else {
            view->scale(1/1.1, 1/1.1);
        }
        event->accept();
    } else {
        // Default pan behavior
        QMainWindow::wheelEvent(event);
    }
}

void MainWindow::navigateBack() {}
void MainWindow::expandInstance(const std::string& s) {}
void MainWindow::navigateToModule(const std::string& m) {}
void MainWindow::clearScene() { scene->clear(); }