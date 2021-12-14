#include "MainWindow.h"

namespace cristal {
namespace graphics {

MainWindow::MainWindow() {
  // Set the window size to be 70% of the available screen space
  resize(QDesktopWidget().availableGeometry(this).size() * 0.7);

  QWidget *centralWidget = new QWidget();
  QVBoxLayout *centralWidgetLayout = new QVBoxLayout();
  centralWidgetLayout->setMargin(0);
  centralWidgetLayout->setSpacing(0);
  centralWidget->setLayout(centralWidgetLayout);
  setCentralWidget(centralWidget);

  createMenus();

  setWindowTitle(QString::fromStdString(title));
}

void MainWindow::createMenus() {
  // File
  fileMenu = menuBar()->addMenu(tr("&Fichier"));
  QAction *exitAction = new QAction(tr("Quitter"));
  fileMenu->addAction(exitAction);
  connect(exitAction, SIGNAL(triggered()), this, SLOT(close()));

  // Tools
  toolsMenu = menuBar()->addMenu(tr("&Outils"));
  signalMapper = new QSignalMapper(toolsMenu);

  connect(toolsMenu, &QMenu::aboutToShow, this, &MainWindow::on_tools_click);
}

void MainWindow::on_tools_click() {
  toolsMenu->clear();
  QAction *separatorText = new QAction(tr("Outil 1"));
  separatorText->setEnabled(false);
  toolsMenu->addAction(separatorText);
  toolsMenu->addSeparator();

  toolsMenu->addSeparator();
  addConfigAction = new QAction(tr("Outil 2"));
  toolsMenu->addAction(addConfigAction);
  connect(addConfigAction, &QAction::triggered, this,
          &MainWindow::on_outil_2_click);
  outil3 = new QAction(tr("Outil3"));
  toolsMenu->addAction(outil3);
  connect(outil3, &QAction::triggered, this, &MainWindow::on_outil_3_click);
}

void MainWindow::on_outil_2_click() {
  QString fileName = QFileDialog::getOpenFileName(
      this, "Coucou", QDir::currentPath(),
      tr("Fichier Texte (*.txt);;Fichier de configuration (*.cnf)"));
  if (!fileName.isEmpty()) {
    std::string path = fileName.toStdString();
  }
}

void MainWindow::on_outil_3_click() {}


}  // namespace graphics
}  // namespace cristal
