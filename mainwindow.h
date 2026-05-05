#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <QGraphicsTextItem>
#include <QGraphicsLineItem>
#include <QComboBox>
#include <QLabel>
#include <QToolBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QMenuBar>
#include <QAction>
#include <QLineEdit>
#include <QCompleter>
#include <QGraphicsItem>
#include <QMenu>
#include <QContextMenuEvent>
#include <map>
#include <string>
#include <vector>

#include "netlistparser.h"
#include "circuitgraph.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void loadNetlistFile(const QString& filename) { loadNetlist(filename); }

    enum class LayoutMode { CppElk, Elkjs };

protected:
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void contextMenuEvent(QContextMenuEvent *event) override;
    
private slots:
    void openFile();
    void zoomIn();
    void zoomOut();
    void resetZoom();
    void fitInView();
    void onModuleChanged(const QString& moduleName);
    void locateNext();
    void locatePrev();
    void locateEnterPressed();
    void onLocateTextChanged(const QString& text);

private:
    QGraphicsScene *scene;
    QGraphicsView *view;
    QComboBox *moduleCombo;
    QComboBox *layoutCombo;
    QLineEdit *locateEdit;
    QCompleter *locateCompleter;
    LayoutMode currentLayoutMode = LayoutMode::CppElk;
    
    Netlist currentNetlist;
    std::string currentModuleName;
    CircuitGraph* circuitGraph;
    std::vector<std::string> navigationHistory;  // stack of previously viewed module names
    QAction* backAction;
    QLabel* breadcrumbLabel;
    QRectF lastGraphBounds;
    bool simplifiedRenderMode = false;

    struct LocateHit {
        CNode* node = nullptr;
        QString searchable;
    };
    std::vector<LocateHit> locateHits;
    std::vector<int> locateResultIndices;
    int locateCursor = -1;
    QGraphicsItem* activeHighlightItem = nullptr;

    void loadNetlist(const QString &filename);
    void drawModule();
    void navigateToModule(const std::string& moduleName);
    void navigateBack();
    void expandInstance(const std::string& fullInstPath);
    void expandInstanceRecursive(const std::string& fullInstPath, const std::string& moduleType);
    void updateBreadcrumb();
    
    // Draw helpers
    void drawEdges();
    void drawInstancePins(CNode* n, double cx, double cy);
    void drawInstance(CNode* n, double cx, double cy);
    void drawExpandedInstance(CNode* n, double cx, double cy);
    void drawPort(const GraphNodeData& data, double x, double y, double w, double h);
    void drawNetJoint(const GraphNodeData& data, double x, double y, double w, double h);
    void clearScene();
    void updateNodeBounds();
    void updateLodVisibility();
    void rebuildLocateIndex();
    void updateLocateResults(const QString& query);
    void focusAndHighlightNode(CNode* n);
    void clearActiveHighlight();
};

#endif // MAINWINDOW_H