/*
 * testNcutW
 *
 * Evan Herbst
 * 10 / 13 / 11
 */

#include <array>
#include <iostream>
#include <algorithm>
#include <iterator>
#include "ncuts/ncutW.h"
using std::vector;
using std::cout;
using std::endl;
using namespace Eigen;

int main()
{
	srand((unsigned int)453); //generate known matrices

	const unsigned int NUM_TESTS = 10;
	const unsigned int MTX_SIZE = 20;
	const unsigned int numClusters = 5;
	const bool debug = false;

	//results of running Shi's code on these matrices
	std::array<vector<unsigned int>, NUM_TESTS> matlabImplResults =
	{{
		vector<unsigned int>{4, 5, 2, 1, 4, 1, 1, 4, 4, 2, 3, 3, 2, 3, 5, 3, 1, 2, 4, 1},
		vector<unsigned int>{3, 4, 3, 3, 3, 1, 5, 2, 1, 1, 3, 2, 5, 1, 1, 4, 5, 2, 3, 4},
		vector<unsigned int>{2, 4, 1, 3, 1, 4, 4, 2, 1, 3, 5, 1, 5, 3, 2, 1, 3, 1, 2, 2},
		vector<unsigned int>{5, 2, 4, 2, 5, 3, 4, 5, 4, 3, 1, 1, 4, 2, 3, 1, 5, 2, 3, 3},
		vector<unsigned int>{3, 1, 1, 5, 5, 2, 5, 2, 4, 3, 3, 2, 5, 3, 4, 5, 5, 3, 1, 2},
		vector<unsigned int>{3, 4, 3, 3, 4, 1, 1, 2, 1, 4, 4, 1, 1, 2, 1, 1, 5, 2, 4, 2},
		vector<unsigned int>{3, 1, 4, 4, 1, 1, 2, 2, 3, 4, 5, 1, 3, 5, 1, 2, 5, 5, 2, 3},
		vector<unsigned int>{3, 5, 2, 3, 5, 2, 1, 1, 2, 4, 4, 5, 1, 4, 4, 5, 1, 5, 3, 2},
		vector<unsigned int>{2, 3, 2, 3, 5, 4, 5, 3, 4, 4, 1, 3, 5, 2, 2, 1, 4, 2, 1, 4},
		vector<unsigned int>{1, 1, 4, 2, 3, 5, 2, 1, 3, 3, 3, 1, 5, 1, 5, 2, 4, 2, 1, 4}
	}};
	//go to 0-based indexing
	for(auto i = matlabImplResults.begin(); i != matlabImplResults.end(); i++)
		for(auto j = (*i).begin(); j != (*i).end(); j++)
			(*j)--;

	for(unsigned int i = 0; i < NUM_TESTS; i++)
	{
		MatrixXd W = (MatrixXd::Random(MTX_SIZE, MTX_SIZE) + MatrixXd::Ones(MTX_SIZE, MTX_SIZE)) / 2;
		W = ((W + W.transpose()) / 2).eval(); //eval() because of aliasing
		if(debug)
			cout << "W = " << endl << W << endl << endl;
		const vector<unsigned int> asgns = ncutW(W, numClusters);
		if(debug)
		{
			cout << "asgns = "; std::copy(asgns.begin(), asgns.end(), std::ostream_iterator<unsigned int>(cout, " ")); cout << endl;
		}
		cout << "ncut value for our result = " << ncutObjective(W, numClusters, asgns) << endl;
		cout << "ncut value for Shi result = " << ncutObjective(W, numClusters, matlabImplResults[i]) << endl;
		cout << "-----------" << endl;
		if(debug)
		{
			int q; std::cin >> q;
		}
	}

	return 0;
}
