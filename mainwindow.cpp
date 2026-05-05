#include "mainwindow.h"
#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QDebug>
#include <QPainterPath>
#include <QMouseEvent>
#include <QGraphicsRectItem>
#include <QGraphicsPolygonItem>
#include <QStringListModel>
#include <QSignalBlocker>
#include <QTimer>
#include <QPen>
#include <functional>
#include <iostream>
#include <limits>
#include <cmath>

namespace {
constexpr int kItemRoleKind = Qt::UserRole + 1;
constexpr int kItemRoleNodeHeight = Qt::UserRole + 2;
constexpr int kItemRoleNodeWidth = Qt::UserRole + 3;

const char* kKindInstanceBody = "instance_body";
const char* kKindInstanceText = "instance_text";
const char* kKindPinGlyph = "pin_glyph";
const char* kKindPinText = "pin_text";
const char* kKindPortBody = "port_body";
const char* kKindPortText = "port_text";
const char* kKindTinyMarker = "tiny_marker";

bool fitDebugEnabled() {
    static const bool enabled = qEnvironmentVariableIsSet("SCHEM_FIT_DEBUG");
    return enabled;
}
}

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
    
    QAction *fitAct = new QAction("Fit All", this);
    connect(fitAct, &QAction::triggered, this, &MainWindow::fitInView);
    toolbar->addAction(fitAct);

    toolbar->addSeparator();
    toolbar->addWidget(new QLabel(" Locate: "));
    locateEdit = new QLineEdit(this);
    locateEdit->setPlaceholderText("Instance/Submodule (current module)");
    locateEdit->setMinimumWidth(280);
    locateCompleter = new QCompleter(QStringList(), this);
    locateCompleter->setCaseSensitivity(Qt::CaseInsensitive);
    locateCompleter->setFilterMode(Qt::MatchContains);
    locateEdit->setCompleter(locateCompleter);
    QObject::connect(locateEdit, &QLineEdit::returnPressed, this, &MainWindow::locateEnterPressed);
    QObject::connect(locateEdit, &QLineEdit::textChanged, this, &MainWindow::onLocateTextChanged);
    toolbar->addWidget(locateEdit);

    QAction *locatePrevAct = new QAction("Prev", this);
    connect(locatePrevAct, &QAction::triggered, this, &MainWindow::locatePrev);
    toolbar->addAction(locatePrevAct);

    QAction *locateNextAct = new QAction("Next", this);
    connect(locateNextAct, &QAction::triggered, this, &MainWindow::locateNext);
    toolbar->addAction(locateNextAct);
    
    toolbar->addSeparator();
    toolbar->addWidget(new QLabel(" Layout: "));
    layoutCombo = new QComboBox();
    layoutCombo->addItem("C++ ELK");
    layoutCombo->addItem("elkjs (Node)");
    layoutCombo->setCurrentIndex(0);
    connect(layoutCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx) {
        currentLayoutMode = (idx == 1) ? LayoutMode::Elkjs : LayoutMode::CppElk;
        drawModule();
    });
    toolbar->addWidget(layoutCombo);

    toolbar->addSeparator();
    backAction = new QAction("◀ Back", this);
    connect(backAction, &QAction::triggered, this, &MainWindow::navigateBack);
    backAction->setEnabled(false);
    toolbar->addAction(backAction);

    toolbar->addSeparator();
    breadcrumbLabel = new QLabel("  ");
    breadcrumbLabel->setStyleSheet("color: #555; font-style: italic;");
    toolbar->addWidget(breadcrumbLabel);
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
    navigationHistory.clear();
    backAction->setEnabled(false);
    breadcrumbLabel->setText("  ");
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
    // Manual combo selection resets drill-down history
    navigationHistory.clear();
    backAction->setEnabled(false);
    currentModuleName = moduleName.toStdString();
    updateBreadcrumb();
    drawModule();
}

void MainWindow::drawModule() {
    clearScene();
    if (currentModuleName.empty()) return;

    circuitGraph->buildFromModule(currentNetlist, currentModuleName);
    if (currentLayoutMode == LayoutMode::Elkjs) {
        circuitGraph->applyLayoutViaElkjs();

        bool hasEdgeGeometry = false;
        for (const auto* e : circuitGraph->edges) {
            if (!e->points.empty()) { hasEdgeGeometry = true; break; }
        }
        if (!hasEdgeGeometry) {
            qWarning() << "elkjs produced no edge geometry; fallback to C++ ELK layout for rendering";
            circuitGraph->applyLayout();
        }
    } else {
        circuitGraph->applyLayout();
    }

    simplifiedRenderMode = (circuitGraph->nodes.size() > 8000 ||
                            circuitGraph->edges.size() > 20000);
    if (simplifiedRenderMode) {
        qWarning() << "Large design render mode enabled:" << "nodes=" << circuitGraph->nodes.size()
                   << "edges=" << circuitGraph->edges.size();
    }

    for (CNode* n : circuitGraph->nodes) {
        QPointF absPos = getAbsolutePos(n);
        double cx = absPos.x() + n->width / 2;
        double cy = absPos.y() + n->height / 2;
        const auto& data = n->data;
        // First pass: draw expanded instance containers (z=-1, behind children)
        if (data.type == CircuitNodeType::ExpandedInstance) {
            drawExpandedInstance(n, cx, cy);
        }
    }

    for (CNode* n : circuitGraph->nodes) {
        QPointF absPos = getAbsolutePos(n);
        double cx = absPos.x() + n->width / 2;
        double cy = absPos.y() + n->height / 2;
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
    updateNodeBounds();
    rebuildLocateIndex();
    updateLodVisibility();
    scene->update();
    fitInView();
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
            auto* pinRect = scene->addRect(px, py, 4, 4, QPen(Qt::black), QBrush(Qt::gray));
            pinRect->setData(kItemRoleKind, kKindPinGlyph);
            pinRect->setData(kItemRoleNodeHeight, h);
            pinRect->setData(kItemRoleNodeWidth, w);
            QGraphicsTextItem* t = scene->addText(QString::fromStdString(p.name));
            t->setScale(0.6);
            if (p.x < w / 2) t->setPos(px + 2, py - 8);
            else t->setPos(px - 15, py - 8);
            t->setData(kItemRoleKind, kKindPinText);
            t->setData(kItemRoleNodeHeight, h);
            t->setData(kItemRoleNodeWidth, w);
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
            auto* pinRect = scene->addRect(px - 2, py - 2, 4, 4, QPen(Qt::black), QBrush(Qt::gray));
            pinRect->setData(kItemRoleKind, kKindPinGlyph);
            pinRect->setData(kItemRoleNodeHeight, h);
            pinRect->setData(kItemRoleNodeWidth, w);
            QGraphicsTextItem* t = scene->addText(QString::fromStdString(inputs[i]));
            t->setScale(0.6); t->setPos(px + 2, py - 8);
            t->setData(kItemRoleKind, kKindPinText);
            t->setData(kItemRoleNodeHeight, h);
            t->setData(kItemRoleNodeWidth, w);
        }
        stepY = h / (outputs.size() + 1);
        for (size_t i = 0; i < outputs.size(); ++i) {
            double py = (y - h/2) + stepY * (i + 1);
            double px = x + w/2;
            auto* pinRect = scene->addRect(px - 2, py - 2, 4, 4, QPen(Qt::black), QBrush(Qt::gray));
            pinRect->setData(kItemRoleKind, kKindPinGlyph);
            pinRect->setData(kItemRoleNodeHeight, h);
            pinRect->setData(kItemRoleNodeWidth, w);
            QGraphicsTextItem* t = scene->addText(QString::fromStdString(outputs[i]));
            t->setScale(0.6); t->setPos(px - 15, py - 8);
            t->setData(kItemRoleKind, kKindPinText);
            t->setData(kItemRoleNodeHeight, h);
            t->setData(kItemRoleNodeWidth, w);
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
    QGraphicsRectItem* rect = scene->addRect(x - w/2, y - h/2, w, h, QPen(Qt::black, 0), QBrush(Qt::NoBrush));
    rect->setData(kItemRoleKind, kKindInstanceBody);
    rect->setData(kItemRoleNodeHeight, h);
    rect->setData(kItemRoleNodeWidth, w);
    
    if (data.type == CircuitNodeType::ModuleInstance && currentNetlist.modules.count(data.moduleType)) {
        rect->setBrush(QColor(200, 220, 255)); 
        rect->setPen(QPen(Qt::darkBlue, 0));
        rect->setData(0, QString::fromStdString(data.moduleType));
        rect->setData(1, QString::fromStdString(data.path)); 
        rect->setCursor(Qt::PointingHandCursor);
        
        // Draw Pins from n->ports or module
        drawInstancePins(n, cx, cy);
    } else if (data.type == CircuitNodeType::ModuleInstance) {
        // External module - still try to draw pins from n->ports
        rect->setBrush(QColor(240, 240, 240)); 
        rect->setPen(QPen(Qt::black, 0));
        if (!n->ports.empty()) {
            drawInstancePins(n, cx, cy);
        }
    } else {
        rect->setBrush(QColor(240, 240, 240)); 
        rect->setPen(QPen(Qt::black, 0));
    }

    QGraphicsTextItem* text = scene->addText(QString::fromStdString(data.label));
    QRectF b = text->boundingRect();
    text->setPos(x - b.width()/2, y - b.height()/2);
    text->setData(kItemRoleKind, kKindInstanceText);
    text->setData(kItemRoleNodeHeight, h);
    text->setData(kItemRoleNodeWidth, w);
}

void MainWindow::drawExpandedInstance(CNode* n, double cx, double cy) {
    const GraphNodeData& data = n->data;
    double w = n->width;
    double h = n->height;
    constexpr double HEADER_H = 25.0;

    // Container background (drawn at z=-1 so children appear on top)
    QPen containerPen(QColor(0, 100, 180), 1.5, Qt::DashLine);
    QBrush containerBrush(QColor(220, 235, 255, 180));
    QGraphicsRectItem* containerRect = scene->addRect(
        cx - w / 2, cy - h / 2, w, h, containerPen, containerBrush);
    containerRect->setZValue(-1.0);
    containerRect->setData(kItemRoleKind, kKindInstanceBody);
    containerRect->setData(kItemRoleNodeHeight, h);
    containerRect->setData(kItemRoleNodeWidth, w);
    // Store module type and path for double-click navigation and context menu
    containerRect->setData(0, QString::fromStdString(data.moduleType));
    containerRect->setData(1, QString::fromStdString(data.path));
    containerRect->setCursor(Qt::PointingHandCursor);

    // Header bar
    QGraphicsRectItem* headerRect = scene->addRect(
        cx - w / 2, cy - h / 2, w, HEADER_H,
        QPen(QColor(0, 80, 160)), QBrush(QColor(80, 140, 210)));
    headerRect->setZValue(-0.5);
    headerRect->setData(0, QString::fromStdString(data.moduleType));
    headerRect->setData(1, QString::fromStdString(data.path));
    headerRect->setCursor(Qt::PointingHandCursor);

    // Label text in header
    QString labelStr = QString::fromStdString(data.label);
    QGraphicsTextItem* text = scene->addText(labelStr);
    text->setDefaultTextColor(Qt::white);
    text->setScale(0.75);
    QRectF tb = text->boundingRect();
    text->setPos(cx - tb.width() * 0.75 / 2.0, cy - h / 2 + 2.0);
    text->setZValue(-0.4);
    text->setData(kItemRoleKind, kKindInstanceText);
    text->setData(kItemRoleNodeHeight, h);
    text->setData(kItemRoleNodeWidth, w);

    // Draw port connectors on the boundary (the same ports as the module has)
    drawInstancePins(n, cx, cy);
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
    shape->setPen(QPen(Qt::black, 0));
    shape->setData(kItemRoleKind, kKindPortBody);
    shape->setData(kItemRoleNodeHeight, h);
    shape->setData(kItemRoleNodeWidth, w);

    auto* marker = scene->addEllipse(-2, -2, 4, 4, QPen(Qt::darkRed), QBrush(Qt::darkRed));
    marker->setPos(x, y);
    marker->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
    marker->setData(kItemRoleKind, kKindTinyMarker);
    marker->setData(kItemRoleNodeHeight, h);
    marker->setData(kItemRoleNodeWidth, w);

    QGraphicsTextItem* text = scene->addText(QString::fromStdString(data.label));
    text->setPos(x - 10, y - 20);
    text->setData(kItemRoleKind, kKindPortText);
    text->setData(kItemRoleNodeHeight, h);
    text->setData(kItemRoleNodeWidth, w);
}

void MainWindow::drawNetJoint(const GraphNodeData& data, double x, double y, double w, double h) {
    scene->addEllipse(x - w/2, y - h/2, w, h, QPen(Qt::black), QBrush(Qt::black));
}

void MainWindow::drawEdges() {
    // For large designs, batch all edges into one path item to avoid per-edge item overhead.
    if (simplifiedRenderMode) {
        QPainterPath merged;
        bool hasAny = false;
        for (CEdge* e : circuitGraph->edges) {
            if (e->points.empty()) continue;
            merged.moveTo(e->points[0]);
            for (size_t i = 1; i < e->points.size(); ++i) {
                merged.lineTo(e->points[i]);
            }
            hasAny = true;
        }
        if (hasAny) {
            scene->addPath(merged, QPen(Qt::black, 1));
        }
        return;
    }

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

void MainWindow::zoomIn() {
    view->scale(1.2, 1.2);
    updateLodVisibility();
}
void MainWindow::zoomOut() {
    view->scale(1/1.2, 1/1.2);
    updateLodVisibility();
}
void MainWindow::resetZoom() {
    view->resetTransform();
    updateLodVisibility();
}
void MainWindow::fitInView() {
    view->resetTransform();
    QRectF target = lastGraphBounds;
    bool usedSceneBounds = false;
    if (target.isNull() || !target.isValid()) {
        target = scene->itemsBoundingRect();
        usedSceneBounds = true;
    }
    if (!target.isNull() && target.isValid()) {
        // Keep scene rect in sync with current target bounds. Otherwise, after switching
        // from a huge module to a smaller one, scroll range clamping can prevent true centering.
        scene->setSceneRect(target);
        if (fitDebugEnabled()) {
            qDebug() << "[FITDBG] before fit module=" << QString::fromStdString(currentModuleName)
                     << "source=" << (usedSceneBounds ? "scene.itemsBoundingRect" : "lastGraphBounds")
                     << "target=" << target
                     << "sceneRect=" << scene->sceneRect();
        }
        view->fitInView(target, Qt::KeepAspectRatio);
        view->scale(0.97, 0.97);
        view->centerOn(target.center());
        if (fitDebugEnabled()) {
            const QPoint vpCenter = view->viewport()->rect().center();
            const QPointF sceneCenter = view->mapToScene(vpCenter);
            const QPointF delta = sceneCenter - target.center();
            qDebug() << "[FITDBG] after fit module=" << QString::fromStdString(currentModuleName)
                     << "sceneCenter=" << sceneCenter
                     << "targetCenter=" << target.center()
                     << "delta=" << delta
                     << "scale=" << view->transform().m11() << view->transform().m22();
        }
    } else if (fitDebugEnabled()) {
        qDebug() << "[FITDBG] skipped fit: invalid target module="
                 << QString::fromStdString(currentModuleName)
                 << "target=" << target;
    }
    updateLodVisibility();
}

void MainWindow::mouseDoubleClickEvent(QMouseEvent *event) {
    // Map window coordinates -> view -> scene
    QPoint viewPos = view->mapFromGlobal(event->globalPos());
    QPointF scenePos = view->mapToScene(viewPos);
    QGraphicsItem* item = scene->itemAt(scenePos, view->transform());

    // Walk up to find a rect item that carries module metadata
    while (item) {
        QGraphicsRectItem* rect = dynamic_cast<QGraphicsRectItem*>(item);
        if (rect) {
            QString moduleType = rect->data(0).toString();
            if (!moduleType.isEmpty() && currentNetlist.modules.count(moduleType.toStdString())) {
                navigateToModule(moduleType.toStdString());
                event->accept();
                return;
            }
        }
        item = item->parentItem();
    }
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
        updateLodVisibility();
        event->accept();
    } else {
        // Default pan behavior
        QMainWindow::wheelEvent(event);
    }
}

void MainWindow::navigateBack() {
    if (navigationHistory.empty()) return;
    currentModuleName = navigationHistory.back();
    navigationHistory.pop_back();

    // Sync combo without triggering onModuleChanged recursion
    moduleCombo->blockSignals(true);
    moduleCombo->setCurrentText(QString::fromStdString(currentModuleName));
    moduleCombo->blockSignals(false);

    backAction->setEnabled(!navigationHistory.empty());
    updateBreadcrumb();
    drawModule();
    fitInView();
}

void MainWindow::navigateToModule(const std::string& moduleName) {
    if (moduleName == currentModuleName) return;
    navigationHistory.push_back(currentModuleName);
    currentModuleName = moduleName;

    moduleCombo->blockSignals(true);
    moduleCombo->setCurrentText(QString::fromStdString(currentModuleName));
    moduleCombo->blockSignals(false);

    backAction->setEnabled(true);
    updateBreadcrumb();
    drawModule();
    fitInView();
}

void MainWindow::updateBreadcrumb() {
    QString crumb;
    for (const auto& m : navigationHistory) {
        crumb += QString::fromStdString(m) + " > ";
    }
    crumb += QString::fromStdString(currentModuleName);
    breadcrumbLabel->setText("  " + crumb);
}

void MainWindow::expandInstance(const std::string& fullInstPath) {
    if (circuitGraph->expandedInstances.count(fullInstPath)) {
        // Collapse: remove this instance and all nested sub-instances
        std::vector<std::string> toRemove;
        for (const auto& s : circuitGraph->expandedInstances) {
            if (s == fullInstPath ||
                (s.size() > fullInstPath.size() &&
                 s.substr(0, fullInstPath.size() + 1) == fullInstPath + ".")) {
                toRemove.push_back(s);
            }
        }
        for (const auto& s : toRemove) circuitGraph->expandedInstances.erase(s);
    } else {
        circuitGraph->expandedInstances.insert(fullInstPath);
    }
    drawModule();
}

void MainWindow::expandInstanceRecursive(const std::string& fullInstPath, const std::string& moduleType) {
    circuitGraph->expandedInstances.insert(fullInstPath);
    if (currentNetlist.modules.count(moduleType)) {
        const Module& mod = currentNetlist.modules.at(moduleType);
        for (const auto& inst : mod.instances) {
            // Only recurse into user-defined modules (not standard cells)
            if (currentNetlist.modules.count(inst.type)) {
                expandInstanceRecursive(fullInstPath + "." + inst.name, inst.type);
            }
        }
    }
}

void MainWindow::contextMenuEvent(QContextMenuEvent* event) {
    QPoint viewPos = view->mapFromGlobal(event->globalPos());
    if (!view->rect().contains(viewPos)) {
        QMainWindow::contextMenuEvent(event);
        return;
    }

    QPointF scenePos = view->mapToScene(viewPos);
    QGraphicsItem* item = scene->itemAt(scenePos, view->transform());

    while (item) {
        QGraphicsRectItem* rect = dynamic_cast<QGraphicsRectItem*>(item);
        if (rect) {
            QString moduleType = rect->data(0).toString();
            QString instPath   = rect->data(1).toString();
            if (!moduleType.isEmpty() && !instPath.isEmpty() &&
                currentNetlist.modules.count(moduleType.toStdString())) {

                QMenu menu(this);
                std::string path    = instPath.toStdString();
                std::string modType = moduleType.toStdString();
                bool isExpanded = circuitGraph->expandedInstances.count(path) > 0;

                if (isExpanded) {
                    menu.addAction("Collapse", [this, path]() {
                        expandInstance(path);
                    });
                } else {
                    menu.addAction("Expand inline", [this, path]() {
                        expandInstance(path);
                    });
                    menu.addAction("Expand all (recursive)", [this, path, modType]() {
                        expandInstanceRecursive(path, modType);
                        drawModule();
                    });
                }
                menu.addSeparator();
                menu.addAction("Go to module schematic", [this, modType]() {
                    navigateToModule(modType);
                });

                menu.exec(event->globalPos());
                event->accept();
                return;
            }
        }
        item = item->parentItem();
    }

    QMainWindow::contextMenuEvent(event);
}
void MainWindow::clearScene() {
    activeHighlightItem = nullptr;
    scene->clear();
}

void MainWindow::updateNodeBounds() {
    if (!circuitGraph || circuitGraph->nodes.empty()) {
        lastGraphBounds = QRectF();
        return;
    }

    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    int edgePointCount = 0;
    int invalidEdgePointCount = 0;

    for (const auto* n : circuitGraph->nodes) {
        QPointF absPos = getAbsolutePos(const_cast<CNode*>(n));
        minX = std::min(minX, absPos.x());
        minY = std::min(minY, absPos.y());
        maxX = std::max(maxX, absPos.x() + n->width);
        maxY = std::max(maxY, absPos.y() + n->height);
    }

    // Include routed edge points so Fit All always covers complete schematic.
    for (const auto* e : circuitGraph->edges) {
        for (const auto& p : e->points) {
            edgePointCount++;
            if (!std::isfinite(p.x()) || !std::isfinite(p.y())) {
                invalidEdgePointCount++;
                continue;
            }
            minX = std::min(minX, p.x());
            minY = std::min(minY, p.y());
            maxX = std::max(maxX, p.x());
            maxY = std::max(maxY, p.y());
        }
    }

    const double margin = 60.0;
    lastGraphBounds = QRectF(minX - margin, minY - margin,
                             (maxX - minX) + 2 * margin,
                             (maxY - minY) + 2 * margin);

    if (fitDebugEnabled()) {
        qDebug() << "[FITDBG] bounds module=" << QString::fromStdString(currentModuleName)
                 << "nodes=" << circuitGraph->nodes.size()
                 << "edges=" << circuitGraph->edges.size()
                 << "edgePoints=" << edgePointCount
                 << "invalidEdgePoints=" << invalidEdgePointCount
                 << "lastGraphBounds=" << lastGraphBounds;
    }
}

void MainWindow::updateLodVisibility() {
    if (!scene || !view) return;

    const double sx = std::abs(view->transform().m11());
    const double sy = std::abs(view->transform().m22());
    const double scale = std::min(sx, sy);

    for (QGraphicsItem* item : scene->items()) {
        const QString kind = item->data(kItemRoleKind).toString();
        if (kind.isEmpty()) continue;

        const double nodeH = item->data(kItemRoleNodeHeight).toDouble();
        const double nodeW = item->data(kItemRoleNodeWidth).toDouble();
        const double pxH = nodeH * scale;
        const double pxW = nodeW * scale;
        const double pxMin = std::min(pxH, pxW);

        if (kind == kKindPinGlyph) {
            item->setVisible(pxH >= 12.0);
        } else if (kind == kKindPinText) {
            item->setVisible(pxH >= 20.0);
        } else if (kind == kKindInstanceText || kind == kKindPortText) {
            item->setVisible(pxH >= 10.0);
        } else if (kind == kKindTinyMarker) {
            item->setVisible(pxMin < 12.0);
        }
    }
}

void MainWindow::rebuildLocateIndex() {
    locateHits.clear();
    locateResultIndices.clear();
    locateCursor = -1;

    QStringList candidates;
    if (!circuitGraph) {
        if (locateCompleter) {
            auto* model = new QStringListModel(candidates, locateCompleter);
            locateCompleter->setModel(model);
        }
        return;
    }

    for (auto* n : circuitGraph->nodes) {
        if (!n) continue;
        if (n->data.type != CircuitNodeType::ModuleInstance && n->data.type != CircuitNodeType::ExpandedInstance) {
            continue;
        }

        QString key = QString::fromStdString(n->data.path);
        QString id = QString::fromStdString(n->data.id);
        QString type = QString::fromStdString(n->data.moduleType);
        QString label = QString::fromStdString(n->data.label);

        QString searchable = key + " " + id + " " + type + " " + label;
        locateHits.push_back({n, searchable});

        if (!key.isEmpty()) candidates << key;
        if (!id.isEmpty()) candidates << id;
        if (!type.isEmpty()) candidates << type;
    }

    candidates.removeDuplicates();
    candidates.sort(Qt::CaseInsensitive);
    if (locateCompleter) {
        auto* model = new QStringListModel(candidates, locateCompleter);
        locateCompleter->setModel(model);
    }
    
    if (locateEdit && !locateEdit->text().trimmed().isEmpty()) {
        updateLocateResults(locateEdit->text());
    }
}

void MainWindow::updateLocateResults(const QString& query) {
    locateResultIndices.clear();
    locateCursor = -1;
    clearActiveHighlight();

    const QString q = query.trimmed();
    if (q.isEmpty()) return;

    for (int i = 0; i < static_cast<int>(locateHits.size()); ++i) {
        const QString& s = locateHits[i].searchable;
        if (s.contains(q, Qt::CaseInsensitive)) {
            locateResultIndices.push_back(i);
        }
    }
}

void MainWindow::focusAndHighlightNode(CNode* n) {
    if (!n) return;
    clearActiveHighlight();

    QPointF absPos = getAbsolutePos(n);
    QRectF r(absPos.x(), absPos.y(), n->width, n->height);
    // Scheme A: reset to a stable zoom before centering, so locate is reliable
    // on very large schematics with huge scene extents.
    view->resetTransform();
    view->scale(1.5, 1.5);
    view->centerOn(r.center());

    auto* hi = scene->addRect(r.adjusted(-6.0, -6.0, 6.0, 6.0), QPen(QColor(220, 20, 60), 0), Qt::NoBrush);
    hi->setZValue(1000.0);
    activeHighlightItem = hi;

    QTimer::singleShot(1300, this, [this, hi]() {
        if (activeHighlightItem == hi) {
            scene->removeItem(hi);
            delete hi;
            activeHighlightItem = nullptr;
        }
    });
}

void MainWindow::clearActiveHighlight() {
    if (!activeHighlightItem) return;
    scene->removeItem(activeHighlightItem);
    delete activeHighlightItem;
    activeHighlightItem = nullptr;
}

void MainWindow::locateNext() {
    const QString txt = locateEdit ? locateEdit->text().trimmed() : QString();
    if (locateResultIndices.empty()) {
        updateLocateResults(txt);
    }
    if (locateResultIndices.empty()) {
        return;
    }

    locateCursor = (locateCursor + 1) % static_cast<int>(locateResultIndices.size());
    const int idx = locateResultIndices[locateCursor];
    focusAndHighlightNode(locateHits[idx].node);
}

void MainWindow::locatePrev() {
    if (locateResultIndices.empty()) {
        updateLocateResults(locateEdit ? locateEdit->text() : QString());
    }
    if (locateResultIndices.empty()) {
        return;
    }

    if (locateCursor < 0) locateCursor = 0;
    locateCursor = (locateCursor - 1 + static_cast<int>(locateResultIndices.size())) % static_cast<int>(locateResultIndices.size());
    const int idx = locateResultIndices[locateCursor];
    focusAndHighlightNode(locateHits[idx].node);
}

void MainWindow::locateEnterPressed() {
    locateNext();
}

void MainWindow::onLocateTextChanged(const QString& text) {
    updateLocateResults(text);
}