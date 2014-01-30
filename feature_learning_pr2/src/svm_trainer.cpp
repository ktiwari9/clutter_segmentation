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

using namespace std;
using namespace dlib;

void load(std::string filename, Eigen::MatrixXf& m)
{

	ifstream input_file(filename.c_str());
	std::vector<float> input_vector;
	//std::cout<<" Reading input file "<<std::endl;
	std::copy(std::istream_iterator<float>(input_file), // this denotes "start of stream"
			std::istream_iterator<float>(),   // this denodes "end of stream"
			std::back_inserter< std::vector< float > >(input_vector));
	//std::cout<<" Printing to screen "<<std::endl;
	//std::copy(input_vector.begin(), input_vector.end(), std::ostream_iterator<float>(std::cout, " "));
	//std::cout<<std::endl;
	//std::cout<<" Converting to eigen map "<<std::endl;
	Eigen::Map<Eigen::MatrixXf> value(input_vector.data(),1,input_vector.size());
	m = value;
	// Test to check if data is being loaded correctly
}

int main(int argc, char **argv)
{
	if(argc < 3)
	{
		std::cout<<" Please provide a filename sequence for features and filename for labels"<<std::endl;
		return -1;
	}
	std::string filename;
	std::ifstream input_stream(argv[1]);
	std::string label_filename(argv[2]);

	std::vector<Eigen::MatrixXf> feature_list;
	int cols;
	while(input_stream.good()){

		std::getline(input_stream, filename);
		if(filename.empty() || filename.at(0) == '#') // skip blank lines or comments
			continue;

		std::cout<<" Processing "<<filename<<std::endl;

		Eigen::MatrixXf new_feature;
		load(filename, new_feature);
		cols = new_feature.cols();
		feature_list.push_back(new_feature);
	}


	std::cout<<" Finished reading feature files, Now reading labels from "<<label_filename<<std::endl;
	// Now read the labels

	std::ifstream label_file(label_filename.c_str());
	std::vector<double> labels;
	std::copy(std::istream_iterator<double>(label_file), // this denotes "start of stream"
			std::istream_iterator<double>(),   // this denodes "end of stream"
			std::back_inserter< std::vector< double > >(labels));

	std::cout<<" Finished reading "<<labels.size()<<" label files"<<std::endl;
	// Now try loading the data in to the native data type

	typedef matrix<double,80,1> sample_type;
	std::vector<sample_type> samples;

	// Defining Kernel Type
	typedef radial_basis_kernel<sample_type> kernel_type;

	for(int i = 0 ; i < feature_list.size() ; i++)
	{
		sample_type new_datum;
		std::cout<<" Saving datum : "<<i<<std::endl;
		for(int j = 0 ; j < cols ; j++)
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

    // Randomizing sample order
    randomize_samples(samples, labels);


    // The nu parameter has a maximum value that is dependent on the ratio of the +1 to -1
    // labels in the training data.  This function finds that value.
    const double max_nu = maximum_nu(labels);

    // here we make an instance of the svm_nu_trainer object that uses our kernel type.
    svm_nu_trainer<kernel_type> trainer;

    std::cout << "doing cross validation" << std::endl;

    typedef matrix<double,1,2> accuracy_statistic;

    double final_gamma = 0.0, final_nu = 0.0, best_paccuracy = 0.0, best_naccuracy = 0.0;
    for (double gamma = 0.00001; gamma <= 1; gamma *= 5)
    {
        for (double nu = 0.00001; nu < max_nu; nu *= 5)
        {
            // tell the trainer the parameters we want to use
            trainer.set_kernel(kernel_type(gamma));
            trainer.set_nu(nu);

            std::cout << "gamma: " << gamma << "    nu: " << nu<<std::endl;
            // Print out the cross validation accuracy for 3-fold cross validation using
            // the current gamma and nu.  cross_validate_trainer() returns a row vector.
            // The first element of the vector is the fraction of +1 training examples
            // correctly classified and the second number is the fraction of -1 training
            // examples correctly classified.
            accuracy_statistic statistic;
            statistic = cross_validate_trainer(trainer, samples, labels, 3);
            std::cout << " Current cross validation accuracy: " << statistic;
            if(statistic(0) >= best_paccuracy && statistic(1) >= best_naccuracy)
            {
            	best_paccuracy = statistic(0);
            	best_naccuracy = statistic(1);
            	final_nu = nu;
            	final_gamma = gamma;
            }
        }
    }


    std::cout<<"Best gamma: "<<final_gamma<<"  Best nu: "<<final_nu<<std::endl;
    // From looking at the output of the above loop it turns out that a good value for nu
    // and gamma for this problem is 0.15625 for both.  So that is what we will use.

    // Now we train on the full set of data and obtain the resulting decision function.  We
    // use the value of 0.15625 for nu and gamma.  The decision function will return values
    // >= 0 for samples it predicts are in the +1 class and numbers < 0 for samples it
    // predicts to be in the -1 class.
    trainer.set_kernel(kernel_type(final_gamma));
    trainer.set_nu(final_nu);

    typedef decision_function<kernel_type> dec_funct_type;
    typedef normalized_function<dec_funct_type> funct_type;

    // Here we are making an instance of the normalized_function object.  This object
    // provides a convenient way to store the vector normalization information along with
    // the decision function we are going to learn.
    funct_type learned_function;
    learned_function.normalizer = normalizer;  // save normalization information
    learned_function.function = trainer.train(samples, labels); // perform the actual SVM training and save the results

    // print out the number of support vectors in the resulting decision function
    std::cout << "\n number of support vectors in our learned_function is "
         << learned_function.function.basis_vectors.size() << std::endl;

    //TODO: cout << "This is a +1 class example, the classifier output is " << learned_function(sample) << endl;

    // We can also train a decision function that reports a well conditioned probability
    // instead of just a number > 0 for the +1 class and < 0 for the -1 class.  An example
    // of doing that follows:
    typedef probabilistic_decision_function<kernel_type> probabilistic_funct_type;
    typedef normalized_function<probabilistic_funct_type> pfunct_type;

    pfunct_type learned_pfunct;
    learned_pfunct.normalizer = normalizer;
    learned_pfunct.function = train_probabilistic_decision_function(trainer, samples, labels, 3);
    // Now we have a function that returns the probability that a given sample is of the +1 class.

    // print out the number of support vectors in the resulting decision function.
    // (it should be the same as in the one above)
    cout << "\n number of support vectors in our learned_pfunct is "
         << learned_pfunct.function.decision_funct.basis_vectors.size() << endl;

    //TODO: use this thing for classification: cout << "This +1 class example should have high probability.  Its probability is: " << learned_pfunct(sample) << endl;

    // Another thing that is worth knowing is that just about everything in dlib is
    // serializable.  So for example, you can save the learned_pfunct object to disk and
    // recall it later like so:
    ofstream fout("saved_svm_function.dat",ios::binary);
    serialize(learned_pfunct,fout);
    fout.close();

    // now lets open that file back up and load the function object it contains
    ifstream fin("saved_svm_function.dat",ios::binary);
    deserialize(learned_pfunct, fin);

    std::cout << "\n cross validation accuracy with only 3 support vectors: "
         << cross_validate_trainer(reduced2(trainer,3), samples, labels, 3);

    // Lets print out the original cross validation score too for comparison.
    std::cout << "cross validation accuracy with all the original support vectors: "
         << cross_validate_trainer(trainer, samples, labels, 3);

    // When you run this program you should see that, for this problem, you can reduce the
    // number of basis vectors down to 10 without hurting the cross validation accuracy.

    // To get the reduced decision function out we would just do this:
    learned_function.function = reduced2(trainer,3).train(samples, labels);
    // And similarly for the probabilistic_decision_function:
    learned_pfunct.function = train_probabilistic_decision_function(reduced2(trainer,3), samples, labels, 3);
}

