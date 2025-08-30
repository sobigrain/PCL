#include "internship.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    internship window;
    window.show();
    return app.exec();
}
