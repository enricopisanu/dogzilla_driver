#include "dogzilla_driver/dogzilla_driver.hpp"

#include <iostream>

int main(){
  DogzillaDriver d;
  int b = d.readBattery();
  std::cout << b;
}