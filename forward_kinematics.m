classdef forward_kinematics
%%  Manipulator Forward Kinematics
%
%   Martin L Pryde MSc
%   last updated 29/01/2021
%   any questions, contact me at martinpryde@ymail.com
%
%   This class features many methods useful in the study and analysis in
%   the forward kinematics of open-chain, rigid-body manipulators. It takes
%   in the joint types (prismatic or revolute) and Denavit-Hartenberg
%   parameters of a manipulator and from these can compute for the user:
%       *   The homogenous transform from one link frame to the next with
%       respect to the base frame
%       *   The pose of the end-effector in the base frame as a function
%       of the manipulator joint angles
%       *   The geometric jacobian of the manipulator
%       *   The analytical jacobian of the manipulator with end-effector
%       orientation expressed either in proper or roll-pitch-yaw euler
%       angles
%       *   Either the geometric or analytical jacobian in the minimum
%       representation of the manipulator pose

    properties
        %   "joint_type" property must be a char column vector with n
        %   elements for n joints with each element being either 'p' or 'r'
        %   for a prismatic or revolute joint respectively. For example, if
        %   your manipulator has the joint configurtion RPRR then the
        %   joint_type property must be initialised as ['r'; 'p'; 'r'; 'r'];
        joint_type
        
        %   The remaining properties are the familiar Denavit-Hartenberg
        %   parameters. These must be entered as either double or sym
        %   column vectors for each parameter.
        theta
        a
        alpha
        d
    end
    
    methods
        function obj = forward_kinematics(joint_type, theta, a, alpha, d)
            %%  Constructor
            %   Enter the required properties as described above to create
            %   an instance of this class
            obj.joint_type = joint_type;
            obj.theta = theta;
            obj.a = a;
            obj.alpha = alpha;
            obj.d = d;
        end
        
        function t = dh_parameters(obj)
            %%  Denavit-Hartenberg Parameter Table
            %   This function creates a simple MATLAB table object for
            %   displaying and verifying the parameters have been entered
            %   correctly. The output is a MATLAB table object displaying
            %   the entered parameters.
            varNames = {'joint number', 'joint type', 'theta', 'a', 'alpha', 'd'};
            n = length(obj.theta);
            joint_number = transpose(1:1:n);
            t = table(joint_number, obj.joint_type, obj.theta, obj.a, obj.alpha, obj.d, ...
                'VariableNames', varNames);
        end
        
        function T_list = homogeneous_transform_list(obj)
            %%   List of Homogeneous Transforms
            %   Computes each transform from one link to the previous for a
            %   complete open-chain, rigid-body manipulator. The output is
            %   a 4nx4 matrix where every fourth entry is the ith transform
            %   for the ith link.
            
            %   get number of links
            n = length(obj.theta);
            
            %   compute first transform
            A1 = obj.homogeneous_transform(obj.theta(1,:), obj.a(1,:), obj.alpha(1,:), obj.d(1,:));
            
            %   initialise matrix to be populated by transforms
            varType = class(obj.theta);
            switch varType
                case 'double'
                    T_list = zeros(4*n,4);
                case 'sym'
                    T_list = sym(zeros(4*n,4));
            end
            
            %   set first entry to the first transform in the sequence
            T_list(1:4,:) = A1;
            
            %   compute remaining transforms using commutative property of
            %   homogeneous transfroms
            i = 2;
            while i < n+1
                j = 4*i;
                Theta = obj.theta(i,1);
                A = obj.a(i,1);
                Alpha = obj.alpha(i,1);
                D = obj.d(i,1);
                A2 = obj.homogeneous_transform(Theta, A, Alpha, D);
                A1 = A1*A2;
                T_list(j-3:j,:) = A1;
                i = i + 1;
            end
            
            %   simplify if entered parameters are of type 'sym'
            switch varType
                case 'double'
                    % do nothing
                case 'sym'
                    T_list = simplify(T_list);
            end
        end
        
        function x = end_effector_pose(obj, angleType)
            %%  End-Effector Pose
            %   The output of this function is the pose of the end-effector
            %   in the base frame. Select from either proper euler-angles 
            %   or roll-pitch-yaw angles as orientation representations by 
            %   setting <angleType> to 'ZYZ' or 'RPY' respectively.
            
            %   get number of links
            n = length(obj.theta);
            
            %   get list of homogeneous transforms
            T_list = obj.homogeneous_transform_list();
            
            %   extract end-effector-position
            p = T_list((4*n-3):(4*n-1),4);
            
            %   extract end-effector rotation matrix orientation
            R = T_list((4*n-3):(4*n-1), 1:3);
            
            %   assign key elements to variables
            r11 = R(1,1);
            r13 = R(1,3);
            r21 = R(2,1);
            r23 = R(2,3);
            r31 = R(3,1);
            r32 = R(3,2);
            r33 = R(3,3);
            
            %   compute euler angles
            switch angleType
                case 'ZYZ'
                    phi = atan2(r23, 213);
                    Theta = atan2(sqrt(r13*r13 + r23*r23), r33);
                    psi = atan2(r32, -r31);
                case 'RPY'
                    phi = atan2(r21, r11);
                    Theta = atan2(-r31, sqrt(r32*r32 + r33*r33));
                    psi = atan2(r32, r33);
                otherwise
                    error('Invalid angle type entered: must be either "ZYZ" or "RPY"');
            end
            
            x = [p; phi; Theta; psi];
        end
        
        function J = geometric_jacobian(obj)
            %%  Geometric Jacobian
            %   This function computes the geometric jacobian of an
            %   open-chain, rigid-body manipulator. The output is, for
            %   redundant manipulators, a 6xn jacobian matrix.
            
            %   get number of joints
            n = length(obj.theta);
            
            %   get list of homogeneous transformations
            T_list = obj.homogeneous_transform_list();
            
            %   extract pose of each joint
            p_e = T_list((4*n-3):(4*n-1),4);
            
            %   initialise arrays to be populated to the correct type and
            %   dimensions
            varType = class(obj.theta);
            switch varType
                case 'double'
                    J = zeros(6,n);
                    p_list = zeros(3,n+1);
                    z_list = [zeros(2,n+1); ones(1,n+1)];
                case 'sym'
                    J = sym(zeros(6,n));
                    p_list = sym(zeros(3,n+1));
                    z_list = sym([zeros(2,n+1); ones(1,n+1)]);
            end
            
            %   populate joint pose list and z-vector list
             for i = 1:1:n
                 j = 4*i;
                 p_list(:,i+1) = T_list(j-3:j-1,4);
                 z_list(:,i+1) = T_list(j-3:j-1,1:3)*z_list(:,i+1);
             end
             
            %   compute geometric jacobian. This is the implementation of
            %   equation 3.30 in the textbook
            for i = 1:1:n
                switch obj.joint_type(i,1)
                    case 'p'
                        J(:,i) = [z_list(:,i); zeros(3,1)];
                    case 'r'
                        p = p_e - p_list(:,i);
                        J(:,i) = [cross(z_list(:,i), p); z_list(:,i)];
                end
            end
            
            switch varType
                case 'double'
                    %   do nothing
                case 'sym'
                    J = simplify(J);
            end
        end
        
        function Ja = analytical_jacobian(obj, phi, theta, angleType)
            %%  Analytical Jacobian
            %   This function computes the analytical jacobian of an
            %   open-chain, rigid-body manipulator. The output is, for
            %   redundant manipulators, a 6xn jacobian matrix. Select from
            %   either proper euler-angles or roll-pitch-yaw angles as
            %   orientation representations by setting <angleType> to 'ZYZ'
            %   or 'RPY' respectively.
            
            %   assign cosine shorthands
            Cp = cos(phi);
            Sp = sin(phi);
            Ct = cos(theta);
            St = sin(theta);
            
            %   build orientation representation transform
            switch angleType
                case 'ZYZ'
                    T = [0, -Sp, Cp*St;
                        0, Cp, Sp*St;
                        1, 0, Ct];
                case 'RPY'
                    T = [1, 0, -St;
                        0, Cp, Ct*Sp;
                        0, -Sp, Cp*Ct];
                otherwise
                    error('Invalid angle type entered: must be either "ZYZ" or "RPY"');
            end
            
            %   build jacobian orientation transform
            Ta = [eye(3,3), zeros(3,3); zeros(3,3), T];
            
            %   obtain analytical jacobian from transform and geometric
            %   jacobian
            Ja = Ta*obj.geometric_jacobian();
            
            varType = class(obj.theta);
            switch varType
                case 'double'
                    %   do nothing
                case 'sym'
                    Ja = simplify(Ja);
            end
        end
        
        function J = jacobian_minimum_representation(obj, jacType, phi, theta, angleType)
            %%  Jacobian in minimum representation form
            %   This function outputs either the geometric or analytical
            %   jacobian in the minimum pose representation form i.e. all
            %   null rows removed.
            
            switch jacType
                case 'geometric'
                    J = obj.geometric_jacobian();
                    J = J(~all(J == 0, 2),:);
                case 'analytical'
                    J = obj.analytical_jacobian(phi, theta, angleType);
                    J = J(~all(J == 0, 2),:);
                otherwise
                    error('invalid input: property "jacType" must be set equal to either "geometric" or "analytical"');
            end
        end
    end
    
    methods (Static)
        function T = homogeneous_transform(theta, a, alpha, d)
            %%  Homogeneous Transform
            %   This function computes the homogeneous transform from the
            %   current frame to the previous frame using the scalar DH
            %   parameters entered as inputs. The output of this function
            %   is a 4x4 coordinate transformation matrix.
            Ct = cos(theta);
            St = sin(theta);
            Ca = cos(alpha);
            Sa = sin(alpha);

            T = [Ct, -St*Ca, St*Sa, a*Ct;
                St, Ct*Ca, -Ct*Sa, a*St;
                0, Sa, Ca, d;
                0, 0, 0, 1];
        end
    end
end