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

%%  base
base.breadth = 500e-3;
base.depth = 500e-3;
base.height = 10e-3;

%%  link 1
link_1.height = 200e-3;
link_1.radius = 50e-3;

%%  link 2
link_2.height = 200e-3;
link_2.radius = 50e-3;

%%  link 3
link_3.height = 200e-3;
link_3.radius = 50e-3;

%%  initial joint positions
q1_init = deg2rad(0);
q2_init = deg2rad(0);
q3_init = deg2rad(0);