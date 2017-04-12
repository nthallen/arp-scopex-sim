CPPFLAGS=-I /home/nort/Exp/SCoPEx/ode/pkg/usr/local/include \
  -I /home/nort/Exp/SCoPEx/ode/include
LDFLAGS=-L/home/nort/Exp/SCoPEx/ode/pkg/usr/local/lib -lode \
	-L/home/nort/Exp/SCoPEx/ode/drawstuff/src -ldrawstuff \
	-lGLU -lGL
CXXLD=$(CXX)
LIBTOOL=libtool
CXXLINK = $(LIBTOOL) $(AM_V_lt) --tag=CXX $(AM_LIBTOOLFLAGS) \
	$(LIBTOOLFLAGS) --mode=link $(CXXLD) $(AM_CXXFLAGS) \
	$(CXXFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@

.PHONY : all
all : demo1 demo2 demo3
demo1 : demo1.o
	$(CXXLINK) demo1.o

demo1.o : demo1.cpp

demo2 : demo2.o
	$(CXXLINK) demo2.o

demo2.o : demo2.cpp

demo3 : demo3.o
	$(CXXLINK) demo3.o

demo3.o : demo3.cpp
