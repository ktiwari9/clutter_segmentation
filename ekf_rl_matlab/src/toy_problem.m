%   Copyright (c) 2013, University of Southern California
%   All rights reserved.
%  
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%   POSSIBILITY OF SUCH DAMAGE.
%  
% 
%    \author Bharath Sankaran
%  
%   @b toy problem to test EKF and SARSA (lambda)
%==========================================================================
function [features reward_matrix] = toy_problem(X,A,A_org,cliques)
%==========================================================================
%
% File: flocking_template.m
% Auth: Bharath Sankaran, University of Southern California
% Desc: A simulation engine for graph manipulation
% Inpt: 
%       X  - A Nx2 matrix corresponding to initial team state,
%            where (N >= 3):
%                  X(i,1) gives the initial X-position of agent i
%                  X(i,2) gives the initial Y-position of agent i
%       Dv - A Nx2 matrix corresponding to the initial components
%            of the directional vector representing the heading of
%            each agent
%       A  - An adjacency matrix defining the fixed team topology
%       A_org  - An adjacency matrix defining the original team topology
%       cliques - Cliques in the graph
%
% Outp: None
%
% Note: It must hold for inputs X, A that
%       size(X,1) == size(A,1) == size(A,2) 
%
%==========================================================================
close all;         % close all open figures

global dx_dy ;
dx_dy= [0.1,-0.1,0.1,-0.1; 0.1,0.1,-0.1,-0.1]';

%==========================================================================
% Define simulation constants
%==========================================================================
CTRL_DT    = 0.01; % Control dt (100Hz)
MAX_ITERS  = 10; % Maximum number of iterations

MAX_VEL_R  = 0.1;  % Maximum translational velocity (m/s)
MAX_ACC_R  = 2.5;  % Maximum translational acceleration (m/s^2)

%==========================================================================
% Generate figure for displaying results
%==========================================================================
fig = figure;
hold on;
set(gca,'box','on');
axis equal;
xlabel('m');
ylabel('m');
set(fig,'WindowStyle','docked');

hp = [];                   % Handle set for agent markers
hv = [];                   % Handle set for quiver objects
hg = [];                   % Handle set for graph edges
ht = zeros(size(X,1),50);  % Handle set for agent tails

%==========================================================================
% Main process control loop
%==========================================================================
%Xv = MAX_VEL_R*Dv; % Agents initially travel at max velocity (w/ 0 acceleration)
Xv = zeros(size(X)); % initialize all agent velocities to zero

[hp,hv] = draw_agent_pose(X,Xv,hp,hv);
hg = draw_graph_embedding(X,A,hg);

%ginput(1);

%==========================================================================
% Initializing the reinforcement (SARSA-Lambda) learning control loop
%==========================================================================


%==========================================================================
% Initializing the Kalman Filter
%==========================================================================
A_state_transition = eye(2*size(X,1));
B_control_matrix = eye(2*size(X,1));
X_state_vector = zeros(2*size(X,1),1); 
X_state_vector(1:2:end) = X(:,1);
X_state_vector(2:2:end) = X(:,2);
H_measurement_model =  A_state_transition;
sigma_covariance = rand(2*size(X,1)).*A_state_transition;
process_noise = rand(2*size(X,1));
measurement_noise = rand(2*size(X,1));

%==========================================================================
% Initializing the Reward learning function for rank learning
%==========================================================================

reward_matrix = zeros(4,MAX_ITERS);
del_max = 10000;

%==========================================================================
% Main process control loop
%==========================================================================
k = 1;
tracking_error = zeros(MAX_ITERS,4);
features = zeros(4,MAX_ITERS,7);

%==========================================================================
% Initial mean distance of matrix
%==========================================================================
mean_dist = sqrt(sum((repmat(mean(X),size(X,1),1) - X).^2,2));

while true
    
    %%%% YOUR CONTROL FOR EACH AGENT! %%%%
    
    %% Finding max vertex
    max_vertex = find(sum(A) == max(sum(A)));
    
    if(length(max_vertex) > 1)
        max_vertex = randperm(length(max_vertex));
        max_vertex = max_vertex(1);
    end
    
    %% Picking a random action
    action = randperm(4); action = action(1);
    fprintf('Current action selected in %d \n',action);
    %waitforbuttonpress;
    
    
    %%%% Log Computed Headings! %%%%
    tracking_error(k,1) = sqrt(mean(mean([X_state_vector(1:2:end),X_state_vector(2:2:end)]) - mean(X)).^2);
    tracking_error(k,2) = sqrt(mean((X_state_vector(1:2:end) - X(:,1)).^2));
    tracking_error(k,3) = sqrt(mean((X_state_vector(2:2:end) - X(:,2)).^2));
    %%%% Update pose estimates! %%%%
    %X = X + Xv*CTRL_DT;         
    X = X + Xv;
    
    if(mod(k,2) ~= 0)    
        prev_adj = A;
    end
    X_old_mean = mean(X);
    
    %%% Computing Features for Rank Learning %%%%%
    features(action,k,:) = compute_features(A,max_vertex,X);
    
    %%% Graph update based on manipulated node %%%      
    [X, Xv, A] = update_manipulated_graph(X,Xv,A,max_vertex,action,cliques);
    
    %======================================================================
    %%%%%% Updating the adjacency matrix for long edges %%%%%%%%%%%%%   
    [index_i index_j] = find(triu(A) == 1);
    dist_matrix = sqrt(sum((X(index_i,:) - X(index_j,:)).^2,2));
    edge_dist_variance = dist_matrix > 3*mean_dist(index_i);
    
    index_i = index_i(edge_dist_variance);
    index_j = index_j(edge_dist_variance);
    linear_index = sub2ind(size(A),index_i,index_j);
    
    A(linear_index) = 0;
    
    if((1/(norm(A) - norm(prev_adj))) > del_max)
        
        reward_matrix(action,k) = (del_max + ((norm(A) - norm(prev_adj))/del_max))*(norm(mean(X)) - norm(X_old_mean));
    else
        reward_matrix(action,k) = (1/(abs(norm(A) - norm(prev_adj))))*(norm(mean(X)) - norm(X_old_mean));
    end
    tracking_error(k,4) = norm((mean(X) - X_old_mean),2);
    
    %%%% Update the plot figure %%%%    
    hg = draw_graph_embedding(X,A,hg);        
    [hp,hv] = draw_agent_pose(X,Xv,hp,hv);
    ht = draw_agent_tails(X,k,ht);
    
    %%%% Updates from Kalman Gain %%%%
    % Fix some of this stuff %
    %---------- Check measurement assumption works for simulation ------%
    %Z_measurement = X_state_vector + 0.01*rand(size(X_state_vector)); % simulated measurement
    Z_measurement = zeros(size(X_state_vector));
    Z_measurement(1:2:end) = X(:,1);Z_measurement(2:2:end) = X(:,2);
    
    Kalman_gain = A_state_transition*sigma_covariance*H_measurement_model'...
        *((measurement_noise + H_measurement_model*sigma_covariance*H_measurement_model')^(-1));
    
    sigma_covariance  = process_noise + A_state_transition*sigma_covariance*A_state_transition'...
        - A_state_transition*sigma_covariance*H_measurement_model'*...
        ((measurement_noise + H_measurement_model*sigma_covariance*H_measurement_model')^(-1))...
        *H_measurement_model*sigma_covariance*A_state_transition';
    
    control_vector = zeros(size(X_state_vector));
    control_vector(1:2:end) = Xv(:,1);
    control_vector(2:2:end) = Xv(:,2);
    X_state_vector = A_state_transition*X_state_vector + B_control_matrix*control_vector +...
        Kalman_gain*(Z_measurement - H_measurement_model*X_state_vector);
    
    %--------- Computing the Kalman Gain -------------%
    
    
    title(strcat('k = ',int2str(k)));
    
    drawnow;
    
    %%%% YOUR CHECK FOR CONVERGENCE! (for Q3) %%%%

    k = k + 1;
    
    %%%% Prevent infinite loop! %%%%
    if k > MAX_ITERS
        fprintf('Maximum number of iterations! (quitting)\n');
        break;
    end
       
end

disp('Done!');

%==========================================================================
% Generate figure showing agent headings
%==========================================================================
figure, hold on;
set(gca,'box','on');
%axis([1 k]);
ylabel('\theta_i (rads)');
xlabel('k (iteration number)');
title('Tracking');

for k = 1:4
    subplot(4,1,k)
    plot(tracking_error(:,k),'k');
end

%figure(2)
%imagesc(reward_matrix);
%title('Progression of reward matrix');

%waitforbuttonpress;

end % function flocking_solution


%==========================================================================
%==========================================================================
%
% Func: disp_agent_pose.m
% Auth: Jason Derenick, University of Pennsylvania
% Desc: A utility function for plotting the neighbor graph
% Inpt: 
%       X    - A Nx2 matrix where the ith row corresponds
%              to the position of agent i
%       Xv   - A Nx2 matrix where the ith row corresponds 
%              to the velocity of agent i
%       hp   - Handles for markers representing agents
%       hv   - Handles for quiver objects
%
% Outp: 
%       hp   - New handles for markers representing agents
%       hv   - New handles for quiver objects
%
%==========================================================================
function features = compute_features(A,max_vertex,X)

features = zeros(7,1);
subgraphs = maximalCliques(A,size(A,2));
clique = [];
for index=1:length(subgraphs)
   sub_clique = subgraphs{index};
   if(sum(sub_clique == max_vertex) > 0)
        clique = [sub_clique clique];
   end
end
 clique = unique(clique);
 
 % clique edge count matrix
 features(1) = sum(sum(triu(A(clique,:)))) / sum(sum(triu(A))); % degree, connectivity feature
 features(2) = length(clique)/size(A,2); % cluster feature
 features(3) = norm(mean(X) - mean(X(clique,:))); % distance feature
 features(4) = max(sum(triu(A(clique,:))));
 features(5) = sum(sum(triu(A(clique,:))) == features(4));
 features(6) = norm(A)/norm(A(clique,:));
 features(7) = norm(mean(X) - mean(X(clique,:)));
 
 %features(7:(7+length(clique)-1)) = sqrt(sum((repmat(mean(X),length(clique),1) - X(clique,:)).^2,2));
 
end
%==========================================================================
%==========================================================================
%
% Func: disp_agent_pose.m
% Auth: Jason Derenick, University of Pennsylvania
% Desc: A utility function for plotting the neighbor graph
% Inpt: 
%       X    - A Nx2 matrix where the ith row corresponds
%              to the position of agent i
%       Xv   - A Nx2 matrix where the ith row corresponds 
%              to the velocity of agent i
%       hp   - Handles for markers representing agents
%       hv   - Handles for quiver objects
%
% Outp: 
%       hp   - New handles for markers representing agents
%       hv   - New handles for quiver objects
%
%==========================================================================
function [hp,hv] = draw_agent_pose(X,Xv,hp,hv)

    if ~isempty(hp), delete(hp); end
    if ~isempty(hv), delete(hv); end
    
    % Draw each agent's position
    hp = plot(X(:,1),X(:,2),'ko','MarkerSize',10);
  
    %for i = 1:size(X,1)
    %   text(X(i,1)+0.035,X(i,2)+0.035,int2str(i)); 
    %end
    
    % Generate directional vector for heading
    Dv = zeros(size(X));
    for i = 1:size(Dv,1)
       Dv(i,:) = 0.25*Xv(i,:)./norm(Xv(i,:));
    end

    % Draw each agent's orientation
    hv = quiver(X(:,1),X(:,2),Dv(:,1),Dv(:,2),'AutoScale','off');        
    
end % function draw_agent_pose

%==========================================================================
%==========================================================================
%
% Func: draw_graph_embedding.m
% Auth: Jason Derenick, University of Pennsylvania
% Desc: A utility function for plotting the neighbor graph
% Inpt: 
%       X  - A Nx2 matrix where the ith row corresponds
%            to the position of agent i
%       A  - A NXN adjacency matrix representing the topology
%       hg - Handle returned from previous call to function 
%
% Outp: 
%       hg - New handle for lines illustrating the embedding
%            of the neighbor graph in R^2.
%
%==========================================================================
function hg = draw_graph_embedding(X,A,hg)

hg(~ishandle(hg)) = [];    
if ~isempty(hg),delete(hg); end
    
    k = 1;
    for i = 1:size(A,1)-1        
        for j = i+1:size(A,1)            
            if A(i,j) ~= 0
                hg(k) = line([X(i,1) X(j,1)],[X(i,2) X(j,2)],'Color','g');
                k = k + 1;
            end            
        end        
    end

end % function draw_graph_embedding

%==========================================================================
%==========================================================================
%
% Func: draw_agent_tails.m
% Auth: Jason Derenick, University of Pennsylvania
% Desc: A utility function for plotting each agent's tail
% Inpt: 
%       X  - A Nx2 matrix where the ith row corresponds
%            to the position of agent i
%       k  - Iteration number of algorithm
%       ht - Handle returned from previous call to function 
%
% Outp: 
%       ht - Handles for points defining each agent's tail
%==========================================================================
function ht = draw_agent_tails(X,k,ht)

    for i = 1:size(X,1)
        
       if mod(k,10) == 0
           ht(i,:) = circshift(ht(i,:),[2 1]);
           if ht(i,end) ~= 0, delete(ht(i,end)); end
           ht(i,end) = plot(X(i,1),X(i,2),'k.','MarkerSize',2);
       end
       
    end   
    
end % fu