/*********************************************************************
*
*  Copyright (c) 2014, Computational Learning and Motor Control Laboratory
*  University of Southern California
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************
/**
 * \author Bharath Sankaran
 *
 * @b SVM trainer. Trains SVMs from file.
 *
 */
#include <iostream>
#include <dlib/svm.h>
#include <Eigen/Dense>
#include "feature_learning/macros_time.hpp"
#include "boost/lexical_cast.hpp"

using namespace std;
using namespace dlib;

typedef matrix<double,78,1> sample_type;


typedef one_vs_all_trainer<any_trainer<sample_type> > ova_trainer;

// Next, we will make two different binary classification trainer objects.  One
// which uses kernel ridge regression and RBF kernels and another which uses a
// support vector machine and polynomial kernels.  The particular details don't matter.
// The point of this part of the example is that you can use any kind of trainer object
// with the one_vs_one_trainer.
typedef polynomial_kernel<sample_type> poly_kernel;


void load(std::string filename, Eigen::MatrixXf& m)
{

	ifstream input_file(filename.c_str());
	std::vector<float> input_vector;
	std::copy(std::istream_iterator<float>(input_file), // this denotes "start of stream"
			std::istream_iterator<float>(),   // this denodes "end of stream"
			std::back_inserter< std::vector< float > >(input_vector));
	Eigen::Map<Eigen::MatrixXf> value(input_vector.data(),1,input_vector.size());
	m = value;
	// Test to check if data is being loaded correctly
}

void process_stream(std::string filenames, std::string label_names, std::vector<Eigen::MatrixXf>& feature_list, std::vector<double>& labels,int &cols){


	std::ifstream input_stream(filenames.c_str());
	std::ifstream label_stream(label_names.c_str());
	std::string filename;

	while(input_stream.good()){

		std::getline(input_stream, filename);
		if(filename.empty() || filename.at(0) == '#') // skip blank lines or comments
			continue;

//		std::cout<<" Processing "<<filename<<"..."<<std::endl;

		Eigen::MatrixXf new_feature;
		load(filename, new_feature);
		cols = new_feature.cols();
		feature_list.push_back(new_feature);
	}

	while(label_stream.good()){

		std::getline(label_stream, filename);
		if(filename.empty() || filename.at(0) == '#') // skip blank lines or comments
			continue;

		std::fstream label_file(filename.c_str(), ios::in);
		double label;
		label_file >> label;
		label_file.close();

		if(label > 0)
		{
			if(filename.find("push_r")!=std::string::npos)
				label=2;
			if(filename.find("push_l")!=std::string::npos)
				label=2;
			if(filename.find("push_f")!=std::string::npos)
			    label=2;

		}
		else{
			label = 3;
		}

		labels.push_back(label);
	}

}

int main(int argc, char **argv)
{
	if(argc < 3)
	{
		std::cout<<" Please provide a filename sequence for features and filename for labels"<<std::endl;
		return -1;
	}

	std::vector<Eigen::MatrixXf> feature_list;
	std::vector<double> labels;
	int cols;

	process_stream(argv[1],argv[2],feature_list,labels,cols);
	std::cout<<" Finished reading "<<feature_list.size()<<" feature files and "<<labels.size()<<" label files"<<std::endl;
	std::cout<<" Number of cols per feature: "<<cols<<std::endl;



	std::vector<sample_type> samples;

	for(int i = 0 ; i < feature_list.size() ; i++)
	{
		sample_type new_datum;
		//std::cout<<" Saving datum : "<<i<<std::endl;
		for(int j = 0 ; j < cols-2 ; j++)
			{
			Eigen::MatrixXf current_sample = feature_list[i];
			new_datum(j) = static_cast<double>(current_sample(0,j));
			}
		samples.push_back(new_datum);
	}


	// Vector Normalizer
	vector_normalizer<sample_type> normalizer;
	// let the normalizer learn the mean and standard deviation of the samples
	normalizer.train(samples);

	// now normalize each sample
	for (unsigned long i = 0; i < samples.size(); ++i)
		samples[i] = normalizer(samples[i]);

	std::cout<<" Randomizing samples for training"<<std::endl;
	randomize_samples(samples, labels);

	// Now try loading the data in to the native data type
	std::cout << "Data loading complete"<<std::endl;


	ova_trainer trainer_a;

	std::cout<<" Setting kernel parameters"<<std::endl;
	static const double gamma[] = {0.000793701,0.00001,0.000793701};
	static const double nu[] = {1.988,1.988,0.000584609};
	static const double poly[] = {2,2,3};

/*	std::vector<double> nu_vec (nu, nu + sizeof(nu) / sizeof(nu[0]) );
	std::vector<double> gamma_vec (gamma, gamma + sizeof(gamma) / sizeof(gamma[0]) );
	std::vector<double> poly_vec (poly, poly + sizeof(poly) / sizeof(poly[0]) );*/

	std::cout<<"-------------------------"<<std::endl;
	std::cout<<"-------One vs all--------"<<std::endl;
	std::cout<<"-------------------------"<<std::endl;

	INIT_PROFILING

    one_vs_all_decision_function<ova_trainer,
            decision_function<poly_kernel>
        > dfb,dfc;

	std::string test_file;
	for(int i = 0;i <3; i++)
	{

		// make the binary trainers and set some parameters
		svm_nu_trainer<poly_kernel> poly_trainer;
		poly_trainer.set_kernel(poly_kernel(gamma[i],nu[i],poly[i]));

		std::cout<<"start_training"<<std::endl;
	    trainer_a.set_trainer(poly_trainer);
		std::string filename("svm_multiclass_"+boost::lexical_cast<std::string>(gamma[i])+"_"+boost::lexical_cast<std::string>(nu[i])+"_"+boost::lexical_cast<std::string>(poly[i])+".dat");
		test_file  = filename;
		one_vs_all_decision_function<ova_trainer> df = trainer_a.train(samples, labels);
		dfb = df;
		std::cout<<"Writing to disk"<<std::endl;
		ofstream fout(filename.c_str(), ios::binary);
		serialize(dfb, fout);
		fout.close();

		MEASURE("SVM NU POLY")

	    RESET
	}

	std::cout<<"Sample test reserializing"<<std::endl;
    // load the function back in from disk and store it in df3.
    ifstream fin(test_file.c_str(), ios::binary);
    deserialize(dfc, fin);


    // Test df3 to see that this worked.
    cout << endl;
    cout << "predicted label: "<< dfc(samples[37])  << ", true label: "<< labels[0] << endl;
    cout << "predicted label: "<< dfc(samples[188]) << ", true label: "<< labels[90] << endl;

    cout << "test serialized function: \n" << test_multiclass_decision_function(dfc, samples, labels) << endl;

}


