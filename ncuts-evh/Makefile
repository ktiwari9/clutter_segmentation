BOOST_INCDIR = /usr/include
ARPACKPP_INCDIR = arpack++/include #arpack++, a c++ wrapper for arpack
EIGEN_INCDIR = /usr/include/eigen3
CXX = g++
LD = g++
CXXFLAGS = -std=c++0x -Wall -g -O3 -Iinclude -I$(BOOST_INCDIR) -I$(ARPACKPP_INCDIR) -I$(EIGEN_INCDIR) -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
LDFLAGS = -g -O3
LIBS = -L/usr/lib -larpack -lblas

TESTS = testSparsifyc testSpmtimesd testDiscretisation testNcutW gaussians

default: libncuts.a $(TESTS)

clean:
	rm -f src/*.o src/tests/*.o *.a $(TESTS)

libncuts.a: src/matrixUtils.o src/discretisation.o src/ncutW.o src/sparsifyc.o src/spmtimesd.o
	ar rv $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^

testSparsifyc: src/tests/testSparsifyc.o libncuts.a
	$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)

testSpmtimesd: src/tests/testSpmtimesd.o libncuts.a
	$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)

testDiscretisation: src/tests/testDiscretisation.o libncuts.a
	$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)

testNcutW: src/tests/testNcutW.o libncuts.a
	$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)

gaussians: src/tests/gaussians.o libncuts.a
	$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)

