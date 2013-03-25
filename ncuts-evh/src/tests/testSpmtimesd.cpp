/*
 * testSpmtimesd
 *
 * Evan Herbst
 * 10 / 14 / 11
 */

#include <cassert>
#include <iostream>
#include "ncuts/spmtimesd.h"
using std::cout;
using std::endl;
using namespace Eigen;

void CHECK_EQ(const MatrixXd& m1, const MatrixXd& m2)
{
	assert(m1.rows() == m2.rows() && m1.cols() == m2.cols());
	cout << "max diff " << (m1 - m2).array().maxCoeff() << endl;
}

/*
 * should say "max diff 0" all the time, or at least very small
 */
int main()
{
	const unsigned int NUM_TESTS = 10;
	const unsigned int MTX_SIZE = 10;

	for(unsigned int i = 0; i < NUM_TESTS; i++)
	{
		VectorXd empty(0, 1);
		VectorXd d1(MTX_SIZE), d2(MTX_SIZE);
		MatrixXd m(MTX_SIZE, MTX_SIZE);
		MatrixXd r1, r2;

		//no d1
		r1 = spmtimesd(m, empty, d2);
		r2 = m * DiagonalMatrix<double, Dynamic, Dynamic>(d2);
		CHECK_EQ(r1, r2);

		//no d2
		r1 = spmtimesd(m, d1, empty);
		r2 = DiagonalMatrix<double, Dynamic, Dynamic>(d1) * m;
		CHECK_EQ(r1, r2);

		//d1 and d2
		r1 = spmtimesd(m, d1, d2);
		r2 = DiagonalMatrix<double, Dynamic, Dynamic>(d1) * m * DiagonalMatrix<double, Dynamic, Dynamic>(d2);
		CHECK_EQ(r1, r2);
	}

	return 0;
}
