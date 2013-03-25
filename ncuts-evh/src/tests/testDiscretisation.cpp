/*
 * testDiscretisation
 *
 * Evan Herbst
 * 10 / 14 / 11
 */

#include <iostream>
#include <algorithm>
#include <iterator>
#include "ncuts/discretisation.h"
using std::vector;
using std::cout;
using std::endl;
using namespace Eigen;

int main()
{
	srand(453u); //TODO after debugging, use time
	const unsigned int NUM_TESTS = 100;
	const unsigned int MTX_ROWS = 20, MTX_COLS = 5;

	for(unsigned int i = 0; i < NUM_TESTS; i++)
	{
		MatrixXd evecs = MatrixXd::Random(MTX_ROWS, MTX_COLS);
		cout << "m0 =" << endl << evecs << endl << endl;
		const vector<unsigned int> asgns = discretisation(evecs);
		cout << "mF =" << endl << evecs << endl << endl;
		cout << "asgns = "; std::copy(asgns.begin(), asgns.end(), std::ostream_iterator<unsigned int>(cout, " ")); cout << endl << endl;
		cout << "------------------------------------------------------" << endl;
		int q; std::cin >> q;
	}

	return 0;
}
