clearvars;
close all;
clc;

[file, path] = uigetfile("*.mat", "Select estimate_pose file");
load(strcat(path, file));

run analyze_impl.m