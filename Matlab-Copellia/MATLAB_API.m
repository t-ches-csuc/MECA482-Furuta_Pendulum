%API that allows Coppelia to interface with Matlab Code

% Make sure to have the server side running in CoppeliaSim: 
% in a child script of a CoppeliaSim scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.

sim=remAPI('remoteApi');       %using remoteApiproto.m file
sim.simxFinish(-1);                 %closes any existing connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5); %begins simulation

%Simulation Configuration Steps
client.simxSynchronous(true);



 if(clientID>-1)
     disp('Successfully Connected');
        
        %%Data requests from simulation here 

    sim.simxFinish(clientID);       %Stops connection 
 else
     disp('Connection Failed');
 end



