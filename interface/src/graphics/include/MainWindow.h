#ifndef SRC_INTERFACE_SRC_GRAPHICS_INCLUDE_MAINWINDOW_INCLUDED_H_
#define SRC_INTERFACE_SRC_GRAPHICS_INCLUDE_MAINWINDOW_INCLUDED_H_

#include <QDebug>
#include <QDesktopWidget>
#include <QDir>
#include <QFileDialog>
#include <QList>
#include <QMainWindow>
#include <QMenuBar>
#include <QSignalMapper>
#include <QString>
#include <QTabWidget>
#include <QVBoxLayout>
#include <iostream>

namespace cristal {
namespace middleware {
class Middleware;
}
}  // namespace cristal

namespace cristal {
namespace graphics {

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow();

  cristal::middleware::Middleware* middleware;

 private slots:
  void on_outil_2_click();
  void on_outil_3_click();
  /**
   * @brief this function is called when the user
   * clicked to show the tools menu, it updates the list
   * of existing ports
   */
  void on_tools_click();

 private:
  void createMenus();
  QMenu* fileMenu;
  QAction* exitAction;
  QMenu* toolsMenu;
  QAction* addConfigAction;
  QAction* outil3;
  QSignalMapper* signalMapper;

  const std::string title = "Title";
};

}  // namespace graphics
}  // namespace cristal

#include "middleware.h"

#endif

