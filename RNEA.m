classdef RNEA
%%  RECURSIVE NEWTON-EULER ALGORITHM
%
%   Martin L Pryde MSc
%   last updated 26/01/2021
%   any questions, contact me at martinpryde@ymail.com
%
%   This class is an implementation of the the algorithm described on pages
%   287-289 of "Robotics: Modelling, Planning and Control" by Bruno
%   Siciliano, Lorenzo Sciavicco, Luigi Villani and Giuseppe Oriolo.
%
%   This class contains methods for calculating the complete forward and
%   inverse dynamics for an open-chain, rigid-body manipulator either
%   numerically or using MATLAB symbolic toolbox. This class can also be
%   used to compute the inertia (B) and coriolis (C) matrices needed to
%   calculate the joint variable accelerations of a manipulator using:
%
%       qddot = inv(B)*(tau - C)
%
%   where tau is the initial wrench acting on the manipulator and qddot is
%   the vector of joint accelerations. These matrices and the methods in
%   this class which compute them can be used as the basis for a
%   computed-torque feedback linearisation controller for a manipulator.

    properties
        %   "joint_type" property must be a char column vector with n
        %   elements for n joints with each element being either 'p' or 'r'
        %   for a prismatic or revolute joint respectively. For example, if
        %   your manipulator has the joint configurtion RPRR then the
        %   joint_type property must be initialised as ['r'; 'p'; 'r'; 'r'];
        joint_type;

        %   "q" is a double or sym column vector of manipultor joint poses
        %   for both prismatic and revolute joints
        q;

        %   "alpha" is a double or sym column vector of twists about the
        %   previous x-axis as described in the Denavit-Hartenberg
        %   convention
        alpha;

        %   "r" is a 3xn matrix with each column being the vector
        %   representing the distance from frame i located at the center of
        %   joint i+1 to the frame i-1 located at the center of joint i.
        r;

        %   "r_bw" is a 3xn matrix with each column being the vector
        %   representing the distance from the frame i located at the
        %   center of joint i+1 to the center of mass of link i located in
        %   frame i-1.
        r_bw;

        %   "kr" is a double or sym column vector of gear reduction ratios
        %   for each respective joint motor. In models where motors are not
        %   considered this vector can be set to ones(n,:) where n is the
        %   number of joints.
        kr;

        %   "z0" is the unit vector of the local z-axis for the current
        %   joint.
        z0;

        %   "zm" is a 3xn matrix with each column being the vector
        %   representing the unit vector of the previous motor's local z
        %   axis in the current frame. For models where motors are not
        %   considered this matrix can be set to [zeros(2,n); ones(1,n)];
        zm;

        %   "m" is a double or sym column vector of the masses of link i
        %   and, for models which consider motors, motor i+1.
        m;

        %   "I" is a 3nx3 matrix where every 3 rows is the moment of
        %   inertia tensor for the ith link taken from that link's center
        %   of mass.
        I;

        %   "Im" is a double or sym column vector of motor moments of
        %   inertia about the current motor's z-axis. For models where
        %   motors are not considered this vector can be set to zeros(n,1).
        Im;
    end

    methods
        function obj = RNEA(joint_type, q, alpha, r, r_bw, kr, zm, m, I, Im)
            %%  Constructor
            %   Enter the required properties as described above to create
            %   an instance of this class
            obj.joint_type = joint_type;
            obj.q = q;
            obj.alpha = alpha;
            obj.r = r;
            obj.r_bw = r_bw;
            obj.kr = kr;
            obj.zm = zm;
            obj.m = m;
            obj.I = I;
            obj.Im = Im;

            %   The class will automatically determine wether the
            %   manipulator model will be computed numerically or
            %   symbolically from the properties you have entered and set
            %   the relevant variables to the repective type
            varType = class(q);
            switch varType
                case 'double'
                    obj.z0 = [0; 0; 1];
                case 'sym'
                    obj.z0 = sym([0; 0; 1]);
            end

        end

        function w = angVel(obj, qdot)
            %%  Link Angular Velocities
            %   This function computes the angular velocity vector of each
            %   link in the manipulator chain. The output is a 3xn matrix
            %   where the ith column is the angular velocity vector of the
            %   ith link.

            %   get number of joints
            n = length(qdot);
            qdot = [0; qdot];

            %   initialise matrix to be populated with angular vecloties to
            %   the correct type and dimensions
            varType = class(qdot);
            switch varType
                case 'double'
                    w = zeros(3,n+1);
                case 'sym'
                    w = sym(zeros(3,n+1));
            end

            %   get list of rotation matrices
            R = rotation_matrix_list(obj);

            %   propagate angular velocities forwards from the base of the
            %   manipulator (frame 0) to the end-effector (frame n). These
            %   are calculated using equation 7.107 in the textbook
            for i = 1:1:n
                j = 3*i;
                switch obj.joint_type(i,:)
                    case 'p'
                        w(:,i+1) = transpose(R(j-2:j,:))*w(:,i);
                    case 'r'
                        w(:,i+1) = transpose(R(j-2:j,:))*(w(:,i) + ...
                            qdot(i+1,1)*obj.z0);
                end
            end

            %   remove initial conditions
            w = w(:, 2:n+1);
        end

        function wdot = angAccel(obj, qdot, qddot, w)
            %%  Link Angular Accelerations
            %   This function computes the angular acceleration vector of each
            %   link in the manipulator chain. The output is a 3xn matrix
            %   where the ith column is the angular acceleration vector of the
            %   ith link.

            %   get number of joints
            n = length(qdot);
            qdot = [0; qdot];
            qddot = [0; qddot];

            %   get list of rotation matrices
            R = rotation_matrix_list(obj);

            %   initialise matrix to be populated with angular accelerations to
            %   the correct type and dimensions
            varType = class(qdot);
            switch varType
                case 'double'
                    wdot = zeros(3,n+1);
                case 'sym'
                    wdot = sym(zeros(3,n+1));
            end

            %   propagate angular accelerations forwards from the base of the
            %   manipulator (frame 0) to the end-effector (frame n). These
            %   are calculated using equation 7.108 in the textbook
            for i = 1:1:n
                j = 3*i;
                switch obj.joint_type(i,:)
                    case 'p'
                        wdot(:,i+1) = transpose(R(j-2:j,:))*wdot(:,i);
                    case 'r'
                        wdot(:,i+1) = transpose(R(j-2:j,:))*(wdot(:,i) + ...
                            qddot(i+1,1)*obj.z0 + ...
                            cross(qdot(i+1,1)*w(:,i),obj.z0));
                end
            end

            %   remove initial conditions
            wdot = wdot(:,2:n+1);
        end

        function wmdot = motorAngAccel(obj, qdot, qddot, w, wdot)
            %%  Motor Angular Accelerations
            %   This function computes the angular acceleration vector of each
            %   motor in the manipulator chain. The output is a 3xn matrix
            %   where the ith column is the angular acceleration vector of the
            %   ith motor.

            %   get number of joints
            n = length(qdot);

            w = [zeros(3,1), w];
            wdot = [zeros(3,1), wdot];

            %   initialise matrix to be populated with angular accelerations to
            %   the correct type and dimensions
            varType = class(qdot);
            switch varType
                case 'double'
                    wmdot = zeros(3,n+1);
                case 'sym'
                    wmdot = sym(zeros(3,n+1));
            end

            %   propagate angular accelerations forwards from the base of the
            %   manipulator (frame 0) to the last motor (frame n-1). These
            %   are calculated using equation 7.111 in the textbook
            for i = 1:1:n
                wmdot(:,i+1) = wdot(:,i) + ...
                    obj.kr(i,1)*qddot(i,1)*obj.zm(:,i) + ...
                    cross(obj.kr(i,1)*qdot(i,1)*w(:,i), obj.zm(:,i));
            end

            %   remove initial conditions
            wmdot = wmdot(:,2:n+1);
        end

        function pddot = transAccel(obj, qdot, qddot, w, wdot, pddot_init)
            %%  Frame Translational Accelerations
            %   This function computes the translational acceleration vector of
            %   each frame in the manipulator chain. The output is a 3xn matrix
            %   where the ith column is the translational acceleration vector of
            %   the ith frame.

            %   get number of frames
            [~,n] = size(w);

            %   list of rotation matrices
            R = rotation_matrix_list(obj);
            qdot = [0; qdot];
            qddot = [0; qddot];
            w = [zeros(3,1), w];
            wdot = [zeros(3,1), wdot];
            r_f = [zeros(3,1), obj.r];

            %   initialise matrix to be populated with translational
            %   accelerations to correct dimensions with desired initial
            %   conditions at base frame
            pddot = [pddot_init, zeros(3,n)];

            %   propagate translational accelerations forwards from the base of
            %   the base frame (frame 0) to the end-effector (frame n-1). These
            %   are calculated using equation 7.109 in the textbook
            for i = 1:1:n
                j = 3*i;
                switch obj.joint_type(i,:)
                    case 'p'
                        ddot = qdot(i+1,:);
                        dddot = qddot(i+1,:);
                        pddot(:,i+1) = transpose(R(j-2:j,:))*(pddot(:,i) + dddot*obj.z0) + ...
                            cross(2*ddot*w(:,i+1), transpose(R(j-2:j,:))*obj.z0) + ...
                            cross(wdot(:,i+1), r_f(:,i+1)) + ...
                            cross(w(:,i+1), cross(w(:,i+1), r_f(:,i+1)));
                    case 'r'
                        pddot(:,i+1) = transpose(R(j-2:j,:))*pddot(:,i) + ...
                            cross(wdot(:,i+1),r_f(:,i+1)) + ...
                            cross(w(:,i+1), cross(w(:,i+1),r_f(:,i+1)));
                end
            end

            %   remove initial conditions
            pddot = pddot(:,2:n+1);
        end

        function pcddot = transAccelCoM(obj, w, wdot, pddot)
            %%  Link Translational Accelerations about Center of Mass (CoM)
            %   This function computes the translational acceleration vector of
            %   each link in the manipulator chain about it's center of mass.
            %   The output is a 3xn matrix where the ith column is the
            %   translational acceleration vector of the ith link.

            %   get number of joints
            [~,n] = size(w);
            r_b = obj.r_bw;

            %   initialise matrix to be populated with translational accelerations
            %   to the correct type and dimensions
            varType = class(w);
            switch varType
                case 'double'
                    pcddot = zeros(3,n);
                case 'sym'
                    pcddot = sym(zeros(3,n));
            end

            %   propagate translational accelerations forwards from the base of
            %   the base frame (frame 0) to the end-effector (frame n-1). These
            %   are calculated using equation 7.110 in the textbook
            for i = 1:1:n
                pcddot(:,i) = pddot(:,i) + ...
                    cross(wdot(:,i),r_b(:,i)) + ...
                    cross(w(:,i), cross(w(:,i),r_b(:,i)));
            end
        end

        function f = forces(obj, pcddot)
            %%  Link Forces
            %   This function computes the vector of forces acting on
            %   each link in the manipulator chain. The output is a 3xn
            %   matrix where the ith column is the force vector of the
            %   ith link.

            %   get number of joints
            [~,n] = size(pcddot);

            %   get list of rotation matrices
            R = rotation_matrix_list(obj);

            %   initialise matrix to be populated with forces to correct
            %   dimensions
            varType = class(pcddot);
            switch varType
                case 'double'
                    f = zeros(3,n+1);
                case 'sym'
                    f = sym(zeros(3,n+1));
            end

            %   propagate forces backwards from the end-effector (frame n)
            %   to the end-effector base (frame n). These are calculated
            %   using equation 7.112 in the textbook
            for i = n:-1:1
                j = 3*(i+1);
                f(:,i) = R(j-2:j,:)*f(:,i+1) + obj.m(i,1)*pcddot(:,i);
            end

            %   remove end-effector conditions
            f = f(:,1:n);
        end

        function t = moments(obj, qdot, qddot, w, wdot, f)
            %%  Link Forces
            %   This function computes the vector of moments acting on
            %   each link in the manipulator chain. The output is a 3xn
            %   matrix where the ith column is the moment vector of the
            %   ith link.

            %   get number of joints
            [~,n] = size(w);
            r_b = obj.r_bw;

            %   get list of rotation matrices
            R = rotation_matrix_list(obj);
            f = [f, zeros(3,1)];
            qdot = [qdot; 0];
            qddot = [qddot; 0];
            Kr = [obj.kr; 0];
            IM = [obj.Im; 0];
            Zm = [obj.zm, zeros(3,1)];

            %   initialise matrix to be populated with moments to correct
            %   dimensions
            varType = class(w);
            switch varType
                case 'double'
                    t = zeros(3,n+1);
                case 'sym'
                    t = sym(zeros(3,n+1));
            end

            %   propagate moments backwards from the end-effector (frame n)
            %   to the end-effector base (frame n). These are calculated
            %   using equation 7.113 in the textbook
            for i = n:-1:1
                j = 3*i;
                k = 3*(i+1);

                t(:,i) = cross(-f(:,i), (obj.r(:,i) + r_b(:,i))) + ...
                    R(k-2:k,:)*t(:,i+1) + ...
                    cross(R(k-2:k,:)*f(:,i+1), r_b(:,i))...
                    + obj.I(j-2:j,:)*wdot(:,i) + ...
                    cross(w(:,i),obj.I((j-2):j,:)*w(:,i)) + ...
                    Kr(i+1,1)*qddot(i+1,1)*IM(i+1,1)*Zm(:,i+1) + ...
                    cross(Kr(i+1,1)*qdot(i+1,1)*IM(i+1,1)*w(:,i), Zm(:,i+1));
            end

            %   remove end-effector conditions
            t = t(:,1:n);
        end

        function [w, wdot, wmdot, pddot, pcddot] = forward_dynamics(obj, qdot, qddot, pddot_init)
            %%  Forward Dynamics
            %   This function calls each function to give the complete
            %   forward dynamics of an open-chain, rigid-body manipulator
            w = angVel(obj, qdot);
            wdot = angAccel(obj, qdot, qddot, w);
            wmdot = motorAngAccel(obj, qdot, qddot, w, wdot);
            pddot = transAccel(obj, qdot, qddot, w, wdot, pddot_init);
            pcddot = transAccelCoM(obj, w, wdot, pddot);
        end

        function [f, t] = inverse_dynamics(obj, qdot, qddot, w, wdot, pcddot)
            %%  Inverse Dynamics
            %   This function calls each function to give the complete
            %   inverse dynamics of an open-chain, rigid-body manipulator
            f = forces(obj, pcddot);
            t = moments(obj, qdot, qddot, w, wdot, f);
        end

        function tau = wrench(obj, qdot, qddot, pddot_init)
            %%  System Wrench
            %   The function computes the force or moment acting on each
            %   respective prismatic or revolute joint in an open-chain
            %   rigid-body manipulator. The output is a column vector where
            %   the ith element is the wrench acting in the z-axis of the
            %   ith joint.

            %   get number of joints
            n = length(qdot);

            %   get list of rotation matrices
            R = rotation_matrix_list(obj);

            %   initialise vector to be populated with joint wrenches to
            %   correct dimensions and type
            varType = class(qdot);
            switch varType
                case 'double'
                    tau = zeros(n,1);
                case 'sym'
                    tau = sym(zeros(n,1));
            end

            %   call forwards and inverse dynamics to obtain forces and
            %   moments acting on each joint
            [w, wdot, wmdot, ~, pcddot] = forward_dynamics(obj, qdot, qddot, pddot_init);
            [f, t] = inverse_dynamics(obj, qdot, qddot, w, wdot, pcddot);

            %   compute wrench acting on each joint in the chain. These are
            %   calculated using equation 7.114 in the textbook
            for i = 1:1:n
                j = 3*i;
                switch obj.joint_type(i,:)
                    case 'p'
                        tau(i,:) = transpose(f(:,i))*R(j-2:j,:)*obj.z0 + ...
                            obj.kr(i,1)*obj.Im(i,1)*transpose(wmdot(:,i))*obj.zm(:,i);
                    case 'r'
                        tau(i,:) = transpose(t(:,i))*R(j-2:j,:)*obj.z0 + ...
                            obj.kr(i,1)*obj.Im(i,1)*transpose(wmdot(:,i))*obj.zm(:,i);
                end
            end
        end

        function [B, C] = dynamic_matrices(obj, qdot, pddot_init)
            %%  Inertia and Coriolis Matrices
            %   This function computes the inertia (B) and coriolis (C)
            %   matrices of an open-chain, rigid-body manipulator. The
            %   outputs are an nxn inertia matrix and an nx1 coriolis
            %   wrench vector.

            %   get number of joints
            n = length(qdot);

            %   initialise outputs to correct type and dimensions and set
            %   respective initial conditions for each matrix computation
            varType = class(qdot);
            switch varType
                case 'sym'
                    qdot_B = sym(zeros(n,1));
                    pddot_init_B = sym(zeros(3,1));
                    B = sym(zeros(n,n));
                    qddot_C = sym(zeros(n,1));
                case 'double'
                    qdot_B = zeros(n,1);
                    pddot_init_B = zeros(3,1);
                    B = zeros(n,n);
                    qddot_C = zeros(n,1);
            end

            %   each column of the B matrix is computed by calling the
            %   system wrench function with joint velocities and
            %   translational acceleration initial conditions both set to
            %   zero. The joint acceleration vector is initialised with a 1
            %   in the ith row corresponding to the ith column of the
            %   matrix undergoing computation.
            for i = 1:1:n
                switch varType
                    case 'sym'
                        qddot_B = sym(zeros(n,1));
                    case 'double'
                        qddot_B = zeros(n,1);
                end
                qddot_B(i,1) = 1;
                B(:,i) = wrench(obj, qdot_B, qddot_B, pddot_init_B);
            end

            %   the coriolis matrix is calculated with a single call to the
            %   system wrench function with the joint acceleration vector
            %   set to zero
            C = wrench(obj, qdot, qddot_C, pddot_init);
        end

        function R_list = rotation_matrix_list(obj)
            %%  Rotation Matrix List
            %   This function computes the rotation matrices of each link
            %   which transform from the ith frame to the i-1 frame. The
            %   output is a 3nxn matrix where every three rows is the ith
            %   rotation matrix which moves from the current to the previous
            %   frame.

            %   remove prismatic variables from list of joint poses
            Q = zeros(size(obj.q));

            for i = 1:1:length(Q)
                switch obj.joint_type(i,:)
                    case 'p'
                        Q(i,:) = 0.0;
                    case 'r'
                        Q(i,:) = obj.q(i,:);
                end
            end

            Q = [Q; 0];
            Alpha = [obj.alpha; 0];

            %   get number of joints
            n = length(Q);

            %   initialise output to correct dimensions and type
            varType = class(obj.q);
            switch varType
                case 'double'
                    R_list = zeros(3*n,3);
                case 'sym'
                    R_list = sym(zeros(3*n,3));
            end

            %   compute rotation matrices according to the
            %   Denavit-Hartenberg convention described in the textbook in
            %   equation 2.52 on page 64.
            for i = 1:1:n
                Cq = cos(Q(i,1));
                Sq = sin(Q(i,1));
                Ca = cos(Alpha(i,1));
                Sa = sin(Alpha(i,1));

                R = [Cq, -Ca*Sq, Sa*Sq;
                    Sq, Ca*Cq, -Sa*Cq;
                    0, Sa, Ca];

                j = 3*i;
                R_list(j-2:j,:) = R;
            end
        end
    end
end
