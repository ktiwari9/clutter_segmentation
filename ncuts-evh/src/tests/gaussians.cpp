/*
 * gaussians: test ncuts on data with pretty clear ground truth
 *
 * Evan Herbst
 * 10 / 21 / 11
 */

#include <cassert>
#include <vector>
#include <random>
#include <functional>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include "ncuts/ncutW.h"
using std::vector;
using std::cout;
using std::endl;
using boost::lexical_cast;
using Eigen::VectorXd;
using Eigen::MatrixXd;

int main(int argc, char* argv[])
{
	unsigned int _ = 1;
	const unsigned int NUM_TESTS = 100;
	const unsigned int DIMENSION = lexical_cast<unsigned int>(argv[_++]);
	const unsigned int NUM_CLUSTERS = lexical_cast<unsigned int>(argv[_++]);
	const unsigned int PTS_PER_CLUSTER = lexical_cast<unsigned int>(argv[_++]);
	const double SIGMA = lexical_cast<double>(argv[_++]);
	assert(_ == argc);

	std::mt19937 rng;
	std::uniform_real_distribution<double> mudist(-1, 1);
	for(unsigned int i = 0; i < NUM_TESTS; i++)
	{
		vector<vector<std::normal_distribution<double>>> dists(NUM_CLUSTERS); //cluster -> dimension -> dist
		vector<vector<std::function<double ()>>> gens(NUM_CLUSTERS);
		vector<VectorXd> pts(NUM_CLUSTERS * PTS_PER_CLUSTER);
		for(unsigned int j = 0; j < NUM_CLUSTERS; j++)
		{
			dists[j].resize(DIMENSION);
			gens[j].resize(DIMENSION);
			for(unsigned int d = 0; d < DIMENSION; d++)
			{
				const double mu = mudist(rng);
				cout << "mu" << d << " for cluster " << j << " is " << mu << endl;
				dists[j][d] = std::normal_distribution<double>(mu, SIGMA);
				gens[j][d] = std::bind(dists[j][d], std::ref(rng));
			}
		}

		for(unsigned int j = 0, k = 0; j < NUM_CLUSTERS; j++)
			for(unsigned int l = 0; l < PTS_PER_CLUSTER; l++, k++)
			{
				pts[k].resize(DIMENSION);
				for(unsigned int d = 0; d < DIMENSION; d++)
					pts[k][d] = gens[j][d]();
			}

		MatrixXd W(pts.size(), pts.size());
		for(unsigned int k = 0; k < pts.size(); k++)
			for(unsigned int l = 0; l < pts.size(); l++)
				W(k, l) = 1 / std::max((pts[k] - pts[l]).squaredNorm(), 1e-6);
//		cout << "W = " << endl << W << endl << endl;
		const vector<unsigned int> asgns = std::move(ncutW(W, NUM_CLUSTERS));
		double tp = 0, fp = 0, tn = 0, fn = 0;
		for(unsigned int j = 0, k = 0; j < NUM_CLUSTERS; j++)
			for(unsigned int l = 0; l < PTS_PER_CLUSTER; l++, k++)
				for(unsigned int j2 = 0, k2 = 0; j2 < NUM_CLUSTERS; j2++)
					for(unsigned int l2 = 0; l2 < PTS_PER_CLUSTER; l2++, k2++)
					{
						const bool trulySame = (j == j2);
						const bool estSame = (asgns[k] == asgns[k2]);
						if(trulySame)
						{
							if(estSame) tp++;
							else fn++;
						}
						else
						{
							if(estSame) fp++;
							else tn++;
						}
					}
		cout << "tp " << tp << " fp " << fp << " tn " << tn << " fn " << fn << endl;
		cout << "N = " << pts.size() << "; acc " << ((tp + tn) / (tp + tn + fp + fn)) << ", prec " << (tp / (tp + fn)) << ", rec " << (tp / (tp + fp)) << endl;
	}

	return 0;
}
