clear all
close all
clc

%% Change vpa to increase speed

digits(8)


%% Get the chain from base to right hand

% load exampleRobots.mat baxter
baxter = importrobot('baxter.urdf');

lefthand = baxter.Bodies{40};
bodies = {lefthand};
index = 1;
joint_indices = [];

while true
    b = bodies{index};
    if strcmp(b.Name, 'base')
        break
    else
        if ~strcmp(b.Joint.Type, 'fixed')
            joint_indices = [joint_indices, index];
        end
        index = index + 1;
        bodies{index} = b.Parent;
        disp(b.Name)
    end
end

%% Get the joint twist axes
% 
joint_list_in_order = joint_indices(end:-1:1);


% trans_arr = cell(size(joint_list_rev));
% 
% for index = 1:numel(joint_list_rev)
%     ji = joint_list_rev(index);
%     trans = eye(4);
%     while ji < numel(bodies)
%         b = bodies{ji};
%         trans = trans*b.Joint.ChildToJointTransform;
%         trans = trans*b.Joint.JointToParentTransform;
%         ji = ji+1;
%     end
%     trans_arr{index} = trans;
% end
% 

zeroconfig = homeConfiguration(baxter);
for index = 1:numel(zeroconfig)
    zeroconfig(index).JointPosition = 0;
end


twist_arr = cell(size(joint_list_in_order));
for index = 1:numel(joint_list_in_order)
    ji = joint_list_in_order(index);
    b = bodies{ji};
    axis = b.Joint.JointAxis;
    axis_homog = [axis, 0]';
    trans = getTransform(baxter, zeroconfig, b.Name);
    w_homog = trans*axis_homog;
    w = w_homog(1:3);
    q = trans(1:3,4);
    v = -hat(w)*q;
    twist = [v;w];
    twist_arr{index} = sym(twist, 'd');
end

gst0 = getTransform(baxter, zeroconfig, bodies{1}.Name);

%% Compute Jacobian

disp('Computing Jacobian')

jacobian_spatial = sym(zeros([6, numel(twist_arr)]));
theta = sym('theta', [numel(twist_arr),1], 'real');

g0toi = eye(4);
for index = 1:numel(twist_arr)
    disp(index)
    xi = twist_arr{index};
    xi_prime = (adjoint_from_transform(g0toi)*xi);
    gi = (screw_transform(xi, theta(index)));
    g0toi = (g0toi*gi);
    
    jacobian_spatial(:,index) = xi_prime;
end
disp('Computing Jacobian done')
gst = g0toi*gst0;

%% Body Jacobian

disp('Computing body Jacobian')

jacobian_body = (inverse_adjoint_from_transform(gst)*jacobian_spatial);

disp('Computing body jacobian done')
%% Compute Mass Matrix

disp('Computing Mass Matrix')

M = sym(zeros(numel(twist_arr)));

for body_index = (numel(bodies)-1):-1:1
    % Find the first joint in the chain which does not affect the body
    for closest_joint_index = 1:numel(joint_list_in_order)
       if joint_list_in_order(closest_joint_index) < body_index
           break
       end
    end
    
    disp(body_index)
    
    % Relevant Matrices
    gsbi0 = sym(getTransform(baxter, zeroconfig, bodies{body_index}.Name), 'd');
    mi = sym(bodies{body_index}.Mass, 'd');
    Ii = sym(bodies{body_index}.Inertia(1:3), 'd');
    Mi = [eye(3)*mi, zeros(3); zeros(3), diag(Ii)];
    
    
    % Construct the proper partial jacobian
    Ji = sym(zeros([6, numel(twist_arr)]));

    % Fill in all the columns below this joint with the appropriate things
    j = 1;
    while j < closest_joint_index
        
        % First come up with the correct transform for the adjoint
        gji = sym(eye(4));
        for k = j:(closest_joint_index-1)
            xi_k = twist_arr{k};
            gk = (screw_transform(xi_k, theta(k)));
            gji = (gji*gk);
        end
        gji = (gji*gsbi0);

        % Now apply it to the twist
        xi_j = twist_arr{j};
        xi_j_dagger = (inverse_adjoint_from_transform(gji)*xi_j);
        Ji(:,j) = xi_j_dagger;
        j = j+1;
    end
    
    M = (M + (Ji'*Mi*Ji));

end
disp('Computing Mass Matrix done')

%% Compute Coriolis Matrix

disp('Computing Coriolis Matrix')

dtheta = sym('dtheta', [numel(twist_arr),1], 'real');

C = sym(zeros(size(M)));


for i = 1:numel(twist_arr)
    for k = 1:numel(twist_arr)
        disp([i,k])
        
        Cik = 0;
        % Compute the cristoffel terms
        for j = 1:numel(twist_arr)
            disp(j)
            Gamma_ijk = (0.5*( ...
                jacobian(M(i, j), theta(k)) + ...
                jacobian(M(i, k), theta(j)) - ...
                jacobian(M(k, j), theta(i)) ...
                ));
            Cik = Cik + Gamma_ijk * dtheta(j);
        end
        C(i,k) = (Cik);
    end
end
disp('Computing Coriolis Matrix done')

%% Compute Gravity Matrix

disp('Computing Gravity Vector')

% First compute V

V = 0;
gravity = 9.81;

for body_index = (numel(bodies)-1):-1:1
    
    disp(body_index)
    
    gsbi0 = getTransform(baxter, zeroconfig, bodies{body_index}.Name);
    gsbi = eye(4);
    for joint_index = 1:numel(joint_list_in_order)
        if joint_list_in_order(joint_index) < body_index
            break;
        else
            xi_j = twist_arr{joint_index};
            gj = screw_transform(xi_j, theta(joint_index));
            gsbi = gsbi*gj;
        end
    end
    gsbi = gsbi*gsbi0;
    
    hi = gsbi(2,4);
    V = V + bodies{body_index}.Mass*gravity*hi;
end

G = jacobian(V, theta);

disp('Computing Gravity Vector done')

%% Save function

disp('Saving jacobian')
fid = fopen('jacobian_gen.txt', 'wt');
fprintf(fid, ccode(jacobian_spatial));
fclose(fid);

disp('Saving inertia matrix')
fid = fopen('inertia_matrix_gen.txt', 'wt');
fprintf(fid, ccode(M));
fclose(fid);

% disp('Saving coriolis matrix')
% fid = fopen('coriolis_matrix_gen.txt', 'wt');
% fprintf(fid, ccode(C));
% fclose(fid);

disp('Saving gravity vector')
fid = fopen('gravity_matrix_gen.txt', 'wt');
fprintf(fid, ccode(G));
fclose(fid);


%% Functions

function what = hat(w)
    what = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
end

function R = rodrigues_formula(w,a)
    R = eye(3) + hat(w)*sin(a)/norm(w) + hat(w)*hat(w)/(norm(w)^2) * (1-cos(a));
end


function xi_hat = wedge(xi)
    v = xi(1:3);
    w = xi(4:6);
    xi_hat = [hat(w), v; zeros(1,3), 0];
end

function trans = screw_transform(xi, a)
    v = xi(1:3);
    w = xi(4:6);
    R = rodrigues_formula(w,a);
    trans = [R, (eye(3) - R)*(hat(w)*v + w*w'*v*a); zeros(1,3),1];
end

function adj = adjoint_from_transform(trans)
    R = trans(1:3, 1:3);
    p = trans(1:3,4);
    adj = [R, hat(p)*R; zeros(3), R];
end

function adj_i = inverse_adjoint_from_transform(trans)
    R = trans(1:3, 1:3);
    p = trans(1:3,4);
    adj_i = [R', -R'*hat(p); zeros(3), R'];
end


