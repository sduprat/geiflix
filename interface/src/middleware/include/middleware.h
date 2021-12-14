#ifndef SRC_INTERFACE_SRC_MIDDLEWARE_INCLUDE_MIDDLEWARE_H_INCLUDED_
#define SRC_INTERFACE_SRC_MIDDLEWARE_INCLUDE_MIDDLEWARE_H_INCLUDED_

#include <QtWidgets>

#include "MainWindow.h"

namespace cristal {
namespace middleware {

class Middleware {
 public:
  Middleware(int& argc, char** argv);

  /**
   * Démarre le middleware
   *
   * @return la valeur de retour de la fenêtre qt
   */
  int start();

 private:
  QApplication app;

  cristal::graphics::MainWindow mainWindow;
};

}  // namespace middleware
}  // namespace cristal

#endif
