#include "Python.h"

int main()
{
   Py_Initialize();
   PyRun_SimpleString("import pylab");
   PyRun_SimpleString("pylab.plot(range(5))");
   PyRun_SimpleString("pylab.show()");
   Py_Exit(0);
   return 0;
}