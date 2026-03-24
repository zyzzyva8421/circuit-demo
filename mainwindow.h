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
#include <map>
#include <string>

#include "netlistparser.h"
#include "circuitgraph.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    
private slots:
    void openFile();
    void zoomIn();
    void zoomOut();
    void resetZoom();
    void onModuleChanged(const QString& moduleName);

private:
    QGraphicsScene *scene;
    QGraphicsView *view;
    QComboBox *moduleCombo;
    
    Netlist currentNetlist;
    std::string currentModuleName;
    CircuitGraph* circuitGraph;
    std::vector<std::string> navigationHistory;
    QAction* backAction;

    void loadNetlist(const QString &filename);
    void drawModule();
    void navigateToModule(const std::string& moduleName);
    void navigateBack();
    void expandInstance(const std::string& fullInstName);
    
    // Draw helpers
    void drawEdges();
    void drawInstance(CNode* n, double cx, double cy);
    void drawPort(const GraphNodeData& data, double x, double y, double w, double h);
    void drawNetJoint(const GraphNodeData& data, double x, double y, double w, double h);
    void clearScene();
};

#endif // MAINWINDOW_H