cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0048 NEW)
project(arp-scopex-sim VERSION 1.1.0)
link_directories(/usr/local/lib)
add_executable(scopex-sim scopex.cc commandfile.cc model_atmos.cc
  nl_error.c nl_verr.c nldbg.c)
target_link_libraries(scopex-sim ode GLU GL)
install(TARGETS scopex-sim RUNTIME DESTINATION bin)
