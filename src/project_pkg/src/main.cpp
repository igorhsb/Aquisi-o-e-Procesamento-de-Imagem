/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui>
#include <ostream>
#include <QApplication>
#include "../include/project_pkg/mainwindow.h"
#include "ui_mainwindow.h"

/*****************************************************************************
** Main
*****************************************************************************/

using namespace std;

int main(int argc, char **argv) {

  QApplication a(argc, argv);
  MainWindow w(argc, argv);
  w.show();

  return a.exec();
}
