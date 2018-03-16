#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "GlWidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char*argv[], QWidget *parent = 0);
    ~MainWindow();
    Ui::MainWindow *ui;
    ros::AsyncSpinner *spinner;

    //  pthread_t labelsThread;
    //  static void *labelsThreadFunc(void *arg);
    //  void atualiza_labels();
    GLWidget *glViewer,*glViewer2;

    void LaserInit(int viewer);
    void ZedInit(int viewer);
    void AstraInit(int viewer);
    void ImuInit(int viewer);
    void KinectInit(int viewer);
    void stopp();

private Q_SLOTS:
    void on_actionExit_triggered();
    void on_dispositivosCB_activated(int index);
    void on_dispositivosCB_currentIndexChanged(const QString &arg1);
    void on_angSpin_valueChanged(double arg1);
    void on_vModeCB_currentIndexChanged(int index);
    void on_dispositivosCB_2_activated(int index);
    void on_dispositivosCB_2_currentIndexChanged(int index);
    void on_vModeCB_2_currentIndexChanged(int index);
    void on_pushButton_clicked();
    void on_checkBox_stateChanged(int arg1);
};

#endif // MAINWINDOW_H
