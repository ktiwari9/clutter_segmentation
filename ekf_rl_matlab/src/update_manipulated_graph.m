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

function [X, Xv, A] = update_manipulated_graph(X,Xv,A,max_vertex,action,cliques)

global dx_dy;

j = 1;
X_old  = X; 

edges_list2 = cell(length(cliques)-1,1);
unmanipulated_cliques = cell(length(cliques)-1,1);
for i = 1:length(cliques)
    if(sum(cell2mat(cliques{i}) == max_vertex) > 0)
        % updating the position of all vertices in clique by value
        Xv(cell2mat(cliques{i})',:) = repmat(dx_dy(action,:),length(cliques{i}),1);
        X = X + Xv;
        [edges_list1 A] = get_edges(X,A,cliques{i});
    else
        [edges_list2{j} A] = get_edges(X,A,cliques{i});
        unmanipulated_cliques{j} = cliques{i};
        j=j+1;
    end
end

% Now check for intersections and update adjacency matrix
clique_intersected = 0;
for i = 1:length(edges_list2)
    update_adjacency(edges_list1,edges_list2{i});
    if clique_intersected
        fprintf('Updating current clique %d \n',i);
        Xv(cell2mat(unmanipulated_cliques{i})',:) = repmat(dx_dy(action,:),length(unmanipulated_cliques{i}),1);
        X = X + Xv;
    end        
end


% randomly add an edge if unintersected
test_edge_list = edges_list2;
test_edge_list{length(edges_list2)+1} = edges_list1;

if (rand > 0.5)
    
    A_sample_matrix = triu(A) + tril(ones(size(A)));
    A_diagonal_indices = find(A_sample_matrix == 0);
    [i,j] = find(A_sample_matrix == 0);
    index = randperm(length(A_diagonal_indices));
    index = index(1);
    i = i(index); j = j(index);    
    test_edge.clique = X([i j],:);
    for index = 1:length(test_edge_list)
        intersected = test_edge_intersection(test_edge,test_edge_list{index});
    end
    if(~intersected)
        A(i,j) = 1;
        A(j,i) = 1;
    end 
end


%fprintf('Initial state vector :\n');
%disp(X_old);

%fprintf('Final state vector :\n');
%disp(X);

%==========================================================================
%
% Update Adjacency function block
%
%==========================================================================

    function update_adjacency(edge_list_1,edge_list_2)
        
        clique_intersected = 0;
        for val_1 = 1:length(edge_list_1)
            for val_2 = 1:2:size(edge_list_1{val_1}.clique,1)
                line_1 = edge_list_1{val_1}.clique(val_2:val_2+1,1:2);
                
                for val_3 = 1:length(edge_list_2)                    
                    % First check clique nodes
                    for val_4 = 1:2:size(edge_list_2{val_3}.clique,1)
                        line_2 = edge_list_2{val_3}.clique(val_4:val_4+1,1:2);
                        [point,inside] = find_intersection(line_1,line_2);
                        if inside
                            % if a single clique node intersects update
                            % the entire clique
                            clique_intersected = 1;
                        end
                        
                        if clique_intersected
                            break; % if one edge intersects no need to check the rest
                        end
                    end
                    
                    % Now check non clique nodes
                    for val_4 = 1:2:size(edge_list_2{val_3}.non_clique,1)
                        line_2 = edge_list_2{val_3}.non_clique(val_4:val_4+1,1:2);
                        [point,inside] = find_intersection(line_1,line_2);
                        
                        if(A(edge_list_2{val_3}.non_clique(val_4,3),edge_list_2{val_3}.non_clique(val_4+1,3)) ~= 0)
                            if inside
                                
                                A(edge_list_2{val_3}.non_clique(val_4,3),edge_list_2{val_3}.non_clique(val_4+1,3)) = 0; % If a clique edge intersects
                                A(edge_list_2{val_3}.non_clique(val_4+1,3),edge_list_2{val_3}.non_clique(val_4,3)) = 0; % If a clique edge intersects
                                % with a non clique edge remove the non clique edge
                                fprintf('We found a collision with a non clique edge %d %d\n',edge_list_2{val_3}.non_clique(val_4,3)...
                                    ,edge_list_2{val_3}.non_clique(val_4+1,3));
                            end
                        end
                        
                    end
                    
                    
                    
                end
                
            end
        end
        
    end

end


function intersected = test_edge_intersection(edge,edge_list)

line_1 = edge.clique(1:2,1:2);

for i = 1:length(edge_list)
    
    for j = 1:2:size(edge_list{i}.clique,1)
        
        line_2 = edge_list{i}.clique(j:j+1,1:2);
        [point,inside] = find_intersection(line_1,line_2);
        
        if(inside)
            intersected = 1;
            return;
        end
        
    end
    
    for j = 1:2:size(edge_list{i}.non_clique,1)
        
        
        line_2 = edge_list{i}.non_clique(j:j+1,1:2);
        [point,inside] = find_intersection(line_1,line_2);        
        if(inside)
            intersected = 1;
            return;
        end
        
    end
    
    
end

intersected = 0;

end

%==========================================================================
%
% File: get_edges.m
% Auth: Bharath Sankaran, University of Southern California
% Desc: Given a set of points, an adjaceny matrix and a clique it returns
% the set of edges/line segments in the matrix
%% FIX THIS
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

function [edge_list A] = get_edges(X,A,clique)

    clique_list = cell2mat(clique);
    edge_list = cell(length(clique_list),1);
    for i = 1:length(clique_list)
        [edge_list{i} A] = build_list(X,A,i,clique_list);
    end

end


function [edge_list A] = build_list(X,A,index,clique_list)

    nodes = find(A(clique_list(index),:) ~= 0);
    
    clique_nodes = nodes(ismember(nodes,clique_list));
    non_clique_nodes = nodes(~ismember(nodes,clique_list));
    
    % Evaluating clique list
    curr_list = X(clique_nodes,:);
    
    clique_edge_list = zeros(2*size(curr_list,1),3);    
    clique_edge_list(1:2:end,:) = [curr_list,clique_nodes'];    
    clique_edge_list(2:2:end,:) = [repmat(X(clique_list(index),:),size(curr_list,1),1),...
        repmat(clique_list(index),size(curr_list,1),1)];
    
    % Evaluating non_clique list
    curr_list = X(non_clique_nodes,:);    
    non_clique_edge_list = zeros(2*size(curr_list,1),3);    
    non_clique_edge_list(1:2:end-1,:) = [curr_list,non_clique_nodes'];    
    non_clique_edge_list(2:2:end,:) = [repmat(X(clique_list(index),:),size(curr_list,1),1),...
        repmat(clique_list(index),size(curr_list,1),1)];
    % Check intersection between clique and non clique edges    
    new_non_clique_edge_list = [];
    for i = 1:2:size(clique_edge_list,1)
        line1 = clique_edge_list(i:i+1,1:2);
        for j = 1:2:size(non_clique_edge_list,1)
            line2 = non_clique_edge_list(j:j+1,1:2);
            [point,inside] = find_intersection(line1,line2);
            
            if(A(clique_nodes(ceil(i/2)),non_clique_nodes(ceil(j/2))) ~= 0)
                if ~inside
                    new_non_clique_edge_list = [new_non_clique_edge_list;non_clique_edge_list(j:j+1,:)];
                else
                    
                    A(clique_nodes(ceil(i/2)),non_clique_nodes(ceil(j/2))) = 0; % If a clique edge intersects
                    A(non_clique_nodes(ceil(j/2)),clique_nodes(ceil(i/2))) = 0; % If a clique edge intersects
                    % with a non clique edge remove the non clique edge
                    fprintf('We found a collision %d %d\n',clique_nodes(ceil(i/2))...
                        ,non_clique_nodes(ceil(j/2)));
                end
            end
            
        end
    end             
    
    edge_list.clique = clique_edge_list;
    edge_list.non_clique = new_non_clique_edge_list;
    
end

%==========================================================================
%
% File: find_intersection.m
% Auth: Bharath Sankaran, University of Southern California
% Desc: finds the intersection of two line segments
%% FIX THIS
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

function [point,inside] = find_intersection(line1,line2)

slope = @(line) (line(2,2) - line(1,2))/(line(2,1) - line(1,1));
m1 = slope(line1);
m2 = slope(line2);

point.x = [];
point.y = [];

intercept = @(line,m) line(1,2) - m*line(1,1);
b1 = intercept(line1,m1);
b2 = intercept(line2,m2);
xintersect = (b2-b1)/(m1-m2);
yintersect = m1*xintersect + b1;

isPointInside = @(xint,myline) ...
    (xint >= myline(1,1) && xint <= myline(2,1)) || ...
    (xint >= myline(2,1) && xint <= myline(1,1));
inside = isPointInside(xintersect,line1) && ...
         isPointInside(xintersect,line2);
     
     if(inside)
         point.x = xintersect;
         point.y = yintersect;
     end

end

