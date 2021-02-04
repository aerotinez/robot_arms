%%  three-link planar arm
%   see page 69 (nice), fig. 2.20. of Siciliano2009
clear;
clc;

%%  dependencies
addpath('~/robot_arms/');

%%  constants
g = 9.81;
aluminium_density = 2710;
steel_density = 7850;

%%  initial joint positions
q1_init = deg2rad(0);
q2_init = deg2rad(0);
q3_init = deg2rad(0);
