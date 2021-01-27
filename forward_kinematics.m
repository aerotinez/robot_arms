classdef forward_kinematics
    properties
        joint_type
        theta
        a
        alpha
        d
    end
    
    methods
        function obj = forward_kinematics(joint_type, theta, a, alpha, d)
            obj.joint_type = joint_type;
            obj.theta = theta;
            obj.a = a;
            obj.alpha = alpha;
            obj.d = d;
        end
        
        function t = dh_parameters(obj)
            varNames = {'joint number', 'joint type', 'theta', 'a', 'alpha', 'd'};
            n = length(obj.theta);
            joint_number = transpose(1:1:n);
            t = table(joint_number, obj.joint_type, obj.theta, obj.a, obj.alpha, obj.d, ...
                'VariableNames', varNames);
        end
        
        function T_list = kinematic_transform_list(obj)
            n = length(obj.theta);
            A1 = obj.kinematic_transform(obj.theta(1,:), obj.a(1,:), obj.alpha(1,:), obj.d(1,:));
            
            varType = class(obj.theta);
            switch varType
                case 'double'
                    T_list = zeros(4*n,4);
                case 'sym'
                    T_list = sym(zeros(4*n,4));
            end
            
            T_list(1:4,:) = A1;
            
            i = 2;
            while i < n+1
                j = 4*i;
                Theta = obj.theta(i,1);
                A = obj.a(i,1);
                Alpha = obj.alpha(i,1);
                D = obj.d(i,1);
                A2 = obj.kinematic_transform(Theta, A, Alpha, D);
                A1 = A1*A2;
                T_list(j-3:j,:) = A1;
                i = i + 1;
            end
            T_list = simplify(T_list);
        end
        
        function J = geometric_jacobian(obj)
            %   get number of joints
            n = length(obj.theta);
            
            %   get list of kinetic transformations
            T_list = obj.kinematic_transform_list();
            
            %   extract pose of each joint
            p_e = T_list((4*n-3):(4*n-1),4);
            
            %   initialise arrays to be populated to the correct type and
            %   dimensions
            varType = class(obj.theta);
            switch varType
                case 'double'
                    J = zeros(6,n);
                    p_list = zeros(3,n);
                    z_list = [zeros(2,n); ones(1,n)];
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
            
            J = simplify(J);
        end
    end
    
    methods (Static)
        function T = kinematic_transform(theta, a, alpha, d)
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