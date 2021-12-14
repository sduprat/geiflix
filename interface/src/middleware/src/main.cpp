#include <iostream>

#include "middleware.h"

int main(int argc, char** argv) {
  cristal::middleware::Middleware m(argc, argv);
  return m.start();
}
