%% Halo Theme


% This script plays a midi of the Halo theme sans rubato
%% Initialize parameters
clear all
amp = 0.4;
tempo = 50;   %bpm
fs = 8192;
% initialize the root pitch and digital signal starting with root
freq = 329.628;
Song = [];
%% Define notes of the song
%notes are defined relative to the note which came before it. Starts with
%zero because of the nature of the equation used to get relative pitch
%(anything raise to the power of zero is one and we define the first note
%of the song with the rest of the parameters). This just makes the loop
%cleaner.
songnotes = [0 2 1 -1 3 -2 -1 -2 7 2 1 -1 -4 4 -2 -12 3 2 3 2 -3 -2 3 -1 -2 2 -4 2];
%These are the note lengths. Arrays should be equal length for now, but if
%we want to add musical rests, there will be more values in this array than
%the previous.
songryths = [1.5 0.5 0.5 0.5 0.5 0.5 1 2 0.5 0.5 1 0.5 0.5 1.25 1.75 0.5 0.5 0.5 0.5 0.75 1.25 0.5 0.5 0.5 0.5 0.5 0.5 2];

%% Build song (maybe this should be a function?????)

for i = 1:length(songryths)
    freq = freq*(1.059463094359)^songnotes(i);
    Song = [Song amp*sin(2*pi*(freq)*(0:1/fs:(songryths(i)/tempo)*60))];
end

sound(Song)