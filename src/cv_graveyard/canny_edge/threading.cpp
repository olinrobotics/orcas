// thread example
#include <iostream>       // std::cout
#include <thread>         // std::thread

bool stop = false;

void foo() 
{
  // do stuff...
  while(!stop){
    printf("hello\n");
  }
}

void bar(int x)
{
  // do stuff...
  int i = 4;
}

int main() 
{
  std::thread first (foo);     // spawn new thread that calls foo()
  //std::thread second (bar,0);  // spawn new thread that calls bar(0)

  //std::cout << "main, foo and bar now execute concurrently...\n";
  getchar();
  stop = true;

  // synchronize threads:
  first.join();                // pauses until first finishes
  //second.join();               // pauses until second finishes

  std::cout << "foo and bar completed.\n";

  return 0;
}