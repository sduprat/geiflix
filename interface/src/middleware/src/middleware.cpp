#include "middleware.h"

#include <chrono>
#include <thread>
#define SUCCESS "x"
#include <QMetaObject>

namespace cristal {
namespace middleware {

Middleware::Middleware(int& argc, char** argv) : app(argc, argv) {
  mainWindow.middleware = this;
}

int Middleware::start() {
  mainWindow.show();
  return app.exec();
}

}  // namespace middleware
}  // namespace cristal
