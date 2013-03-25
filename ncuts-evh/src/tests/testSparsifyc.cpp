/*
 * testSparsifyc
 *
 * Evan Herbst
 * 10 / 13 / 11
 */

#include <cassert>
#include <iostream>
#include <random>
#include "ncuts/sparsifyc.h"
using std::cout;
using std::endl;
using namespace Eigen;

int main()
{
	const unsigned int NUM_TESTS = 10;
	const unsigned int MTX_SIZE = 10;
	std::mt19937 rng;
	std::uniform_real_distribution<double> dist(0, 1);
	auto rand01 = std::bind(dist, rng);
	for(unsigned int i = 0; i < NUM_TESTS; i++)
	{
		const double threshold = rand01();
		MatrixXd m(MTX_SIZE, MTX_SIZE);
		for(unsigned int j = 0; j < MTX_SIZE; j++)
			for(unsigned int k = 0; k < MTX_SIZE; k++)
				m(j, k) = rand01();

		const SparseMatrix<double> result = sparsify(m, threshold);
		cout << "m =" << endl << m << endl << endl;
		cout << "thresh = " << threshold << endl << endl;
		cout << "sp = " << endl << result << endl << endl;

		for(int l=0; l<MTX_SIZE; ++l)
			for(SparseMatrix<double>::InnerIterator it(result, l); it; ++it)
			{
				const unsigned int j = it.row(), k = it.col();
				if(m(j, k) > threshold) assert(it.value() == m(j, k));
				else assert(it.value() == 0);
			}
	}

	return 0;
}
