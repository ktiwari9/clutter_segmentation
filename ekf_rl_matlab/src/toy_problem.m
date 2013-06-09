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
function toy_problem(X,Dv,A)
%==========================================================================
%
% File: flocking_template.m
% Auth: Jason Derenick, University of Pennsylvania
% Desc: A template m-file for MEAM 620, Project 3
% Inpt: 
%       X  - A Nx2 matrix corresponding to initial team state,
%            where (N >= 3):
%                  X(i,1) gives the initial X-position of agent i
%                  X(i,2) gives the initial Y-position of agent i
%       Dv - A Nx2 matrix corresponding to the initial components
%            of the directional vector representing the heading of
%            each agent
%       A  - An adjacency matrix defining the fixed team topology
%
% Outp: None
%
% Note: It must hold for inputs X, A that
%       size(X,1) == size(A,1) == size(A,2)
%
%==========================================================================
close all;         % close all open figures

%==========================================================================
% Define simulation constants
%==========================================================================
CTRL_DT    = 0.01; % Control dt (100Hz)
MAX_ITERS  = 100; % Maximum number of iterations

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
Xv = MAX_VEL_R*Dv; % Agents initially travel at max velocity (w/ 0 acceleration)

[hp,hv] = draw_agent_pose(X,Xv,hp,hv);
hg = draw_graph_embedding(X,A,hg);

%ginput(1);

%==========================================================================
% Main process control loop
%==========================================================================
k = 1;

while true
    
    %%%% YOUR CONTROL FOR EACH AGENT! %%%%
    
    %% Finding max vertex
    max_vertex = find(sum(A) == max(sum(A)));
    
    if(length(max_vertex) > 1)
        max_vertex = randperm(length(max_vertex));
        max_vertex = max_vertex(1);
    end
    
    %%%% Log Computed Headings! %%%%
    log_theta(k,:) = atan2(Xv(:,2),Xv(:,1))'; %#ok<AGROW>
    
    %%%% Update pose estimates! %%%%
    X = X + Xv*CTRL_DT;         

    %%%% Update the plot figure %%%%
    [hp,hv] = draw_agent_pose(X,Xv,hp,hv);
    ht = draw_agent_tails(X,k,ht);
    hg = draw_graph_embedding(X,A,hg);        
    
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
axis([1 k -pi pi]);
ylabel('\theta_i (rads)');
xlabel('k (iteration number)');
title('Convergence of Agent Headings');

for k = 1:size(X,1)
    plot(log_theta(:,k),'k');
end

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