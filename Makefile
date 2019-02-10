CPPFLAGS=-I /home/nort/Exp/SCoPEx/PORT/ode/pkg/usr/local/include \
  -I /home/nort/Exp/SCoPEx/PORT/ode/include
LDFLAGS=-L/home/nort/Exp/SCoPEx/PORT/ode/pkg/usr/local/lib -lode \
	-L/home/nort/Exp/SCoPEx/PORT/ode/drawstuff/src -ldrawstuff \
	-lGLU -lGL -L/usr/local/lib -lnort
MA_LDFLAGS=
CXXFLAGS=-g
CXXLD=$(CXX)
LIBTOOL=libtool
CXXLINK = $(LIBTOOL) $(AM_V_lt) --tag=CXX $(AM_LIBTOOLFLAGS) \
	$(LIBTOOLFLAGS) --mode=link $(CXXLD) $(AM_CXXFLAGS) \
	$(CXXFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@

.PHONY : all clean
all : scopex

scopex : scopex.o commandfile.o model_atmos.o
	$(CXXLINK) scopex.o commandfile.o model_atmos.o
scopex.o : scopex.cpp
commandfile.o : commandfile.cpp

Sdebug : Sdebug.o
	$(CXXLINK) Sdebug.o
Sdebug.o : Sdebug.cpp

clean :
	rm -f *.o Sdebug scopex
