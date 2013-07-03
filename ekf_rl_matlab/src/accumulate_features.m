%   Copyright (c) 2013, University of Southern California
%   All rights reserved.
%  
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%   POSSIBILITY OF SUCH DAMAGE.
%  
% 
%    \author Bharath Sankaran
%  
%   @b Accumulate features
%==========================================================================

%==========================================================================
% Loading datasets
%==========================================================================
clc
clear all;
load example-A.dat;
load example-Aorg.mat;
load example-X.dat;
load cliques;


features_matrix  = zeros(4,10*100,7);
rewards_matrix = zeros(4,10*100);
reward_count = 1;
%feature_count = 1;

for i = 1:100
   
    [features rewards] = toy_problem(example_X,example_A,example_Aorg,cliques);    
    fprintf('Finished one training round');
    
    rewards_matrix(:,reward_count:reward_count+9) = rewards;
        
    features_matrix(:,reward_count:reward_count+9,:) = features;
    reward_count = reward_count + 10;
end

% Save Learning data
save('training_data.mat','rewards_matrix','features_matrix');