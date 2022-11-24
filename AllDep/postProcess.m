
clc
close all
clear

addpath ppLibrary\
load("data\PPallRes.mat")
SaveData = false; %set to true if you want to create csv files with the data
[height, width] = size(AllPPsol);

%initalise variables which are used in the loops
D             = zeros(height,8);
Dtemp         = zeros(1,8);
curv          = zeros(height,8);
cumArcLength  = zeros(height,8);
intPaths{1,8} = [];
ISdH          = zeros(height,8);
IAdH          = zeros(height,8);
meanD         = zeros(3,8);
stdD          = zeros(3,8);
meanCurv      = zeros(3,8);
stdCurv       = zeros(3,8);
meanCAL       = zeros(3,8);
stdCAL        = zeros(3,8);
meanIAdH      = zeros(3,8);
stdIAdH       = zeros(3,8);
meanISdH      = zeros(3,8);
stdISdH       = zeros(3,8);


for j = 1:width
    for i = 1:height
        %extract the solutions in iteration j,i
        tempData = AllPPsol(i,j).data;

        %collect the paths of both the optimised and non optimsied pats in
        %a cell variable
        curData{1,1} = tempData{2,2}.pos; %optimised A*
        curData{2,1} = tempData{3,2}.pos; %optimised Theta*
        curData{3,1} = tempData{4,2}.pos; %optimised Hybrid A*
        curData{4,1} = tempData{5,2}.pos; %optimised RRT*

        curData{5,1} = tempData{2,3}; %A* non opt
        curData{6,1} = tempData{3,3}; %Theta* non opt
        curData{7,1} = tempData{4,3}; %Hybrid A* non opt
        curData{8,1} = tempData{5,3}; %RRT* non opt


        %% Distance
        for n = 1:length(curData)
            Dtemp(1,n) = pathDistance(curData{n,1}(:,1), curData{n,1}(:,2));
        end
        % normalise with respect to opt Theta* and save it in D.
        % Dtemp will be used to compute the curvature as well so we don't
        % want to overwrite it.
        D(i,1:8) = Dtemp(1,1:8) ./ Dtemp(1,2);

        %% Curvature
        for n = 1:length(curData)
            % we skip A*, Theta* and RRT* as these are made up of straight
            % line segements, thus it will have a radius of curvature of inf

            [curv(i,n), cumArcLength(i,n), intPath] = compCurv (Dtemp(1,n), curData{n,1});
            intPaths{1,n} = intPath;
            if (n == 5 || n == 6 || n == 8)
                curv(i,n) = NaN;
            end
        end
        %normalise with respect to opt Theta*
        curv (i,1:8) = curv(i,1:8) ./ curv(i,2);

        %% Compute the integral of absolute heading derivative
        for n = 1:length(curData)
            [IAdH(i,n) , ISdH(i,n)] = IntSdH(curData{n,1}(:,3));
        end

        IAdH(i,1:8) = IAdH(i,1:8) ./IAdH(i,2);
        ISdH(i,1:8) = ISdH(i,1:8) ./ISdH(i,2);

    end
    %compute the mean and std for all metrics for 100 itr
    meanD(j,:) = mean(D);
    stdD(j,:) = std(D);

    meanCurv(j,:) = mean(curv);
    stdCurv(j,:) = std(curv);

    meanCAL(j,:) = mean(cumArcLength);
    stdCAL(j,:) = std(cumArcLength);

    meanIAdH(j,:) = mean(IAdH);
    stdIAdH(j,:) = std(IAdH);

    meanISdH(j,:) = mean(ISdH);
    stdISdH(j,:) = std(ISdH);
end
%compute average path decreased
avgPdec = zeros(1,length(meanD)/2);
for i = 1:length(meanD)/2
    avgPdec(i) = mean(1 - (meanD(:,i) ./ meanD(:,i+4) ));
end


%% Plot the results
figure(11)
clf
x = categorical({'0 Obs', '50 Obs', '100 Obs'});
x = reordercats(x,{'0 Obs', '50 Obs', '100 Obs'});
hold on
b = bar(x, meanD);
b(1,8).FaceColor = [1, 0.06, 0.65];
hold off
title ('Path length')
legend ('Opt. A*', 'Opt. Theta*', 'Opt. Hybrid A*', 'Opt. RRT*',...
    'A*', 'Theta*', 'Hybrid A*', 'RRT*', 'Location','northeastoutside')

figure(12)
clf
b = bar(x, meanCurv);
b(1,8).FaceColor = [1, 0.06, 0.65];
title ('Curvature')
legend ('Opt. A*', 'Opt. Theta*', 'Opt. Hybrid A*', 'Opt. RRT*',...
    'A*', 'Theta*', 'Hybrid A*', 'RRT*', 'Location','northeastoutside')

figure(13)
clf
b = bar(x, meanIAdH);
b(1,8).FaceColor = [1, 0.06, 0.65];
title ('IAT')
legend ('Opt. A*', 'Opt. Theta*', 'Opt. Hybrid A*', 'Opt. RRT*',...
    'A*', 'Theta*', 'Hybrid A*', 'RRT*', 'Location','northeastoutside')



if SaveData
    %% save to csv files
    mazeName = {'Empty maze';'Maze50';'Maze100'};
    mazeNr = [1; 2; 3];

    AData = table(meanD(:,1), stdD(:,1), meanCurv(:,1), stdCurv(:,1),...
        meanIAdH(:,1), stdIAdH(:,1), meanISdH(:,1), stdISdH(:,1), ...
        meanD(:,5), stdD(:,5), meanCurv(:,5), stdCurv(:,5),...
        meanIAdH(:,5), stdIAdH(:,5), meanISdH(:,5), stdISdH(:,5), mazeNr);
    AData.Properties.VariableNames(1:17) = {'meanD_o', 'stdD_o', 'meanCurv_o',...
        'stdCurv_o', 'meanIAdH_o', 'stdIAdH_o', 'meanISdH_o', 'stdISdH_o',...
        'meanD_no','stdD_no', 'meanCurv_no', 'stdCurv_no', 'meanIAdH_no',...
        'stdIAdH_no', 'meanISdH_no', 'stdISdH_no', 'Maze'};
    writetable(AData,'data/AData.csv');


    thetaData = table(meanD(:,2), stdD(:,2), meanCurv(:,2), stdCurv(:,2),...
        meanIAdH(:,2), stdIAdH(:,2), meanISdH(:,2), stdISdH(:,2), ...
        meanD(:,6), stdD(:,6), meanCurv(:,6), stdCurv(:,6),...
        meanIAdH(:,6), stdIAdH(:,6), meanISdH(:,6), stdISdH(:,6), mazeNr);
    thetaData.Properties.VariableNames(1:17) = {'meanD_o', 'stdD_o', 'meanCurv_o',...
        'stdCurv_o', 'meanIAdH_o', 'stdIAdH_o', 'meanISdH_o', 'stdISdH_o',...
        'meanD_no','stdD_no', 'meanCurv_no', 'stdCurv_no', 'meanIAdH_no',...
        'stdIAdH_no', 'meanISdH_no', 'stdISdH_no', 'Maze'};
    writetable(thetaData,'data/thetaData.csv');

    HybridAData = table(meanD(:,3), stdD(:,3), meanCurv(:,3), stdCurv(:,3),...
        meanIAdH(:,3), stdIAdH(:,3), meanISdH(:,3), stdISdH(:,3), ...
        meanD(:,7), stdD(:,7), meanCurv(:,7), stdCurv(:,7),...
        meanIAdH(:,7), stdIAdH(:,7), meanISdH(:,7), stdISdH(:,7), mazeNr);
    HybridAData.Properties.VariableNames(1:17) = {'meanD_o', 'stdD_o', 'meanCurv_o',...
        'stdCurv_o', 'meanIAdH_o', 'stdIAdH_o', 'meanISdH_o', 'stdISdH_o',...
        'meanD_no','stdD_no', 'meanCurv_no', 'stdCurv_no', 'meanIAdH_no',...
        'stdIAdH_no', 'meanISdH_no', 'stdISdH_no', 'Maze'};
    writetable(HybridAData,'data/HybridAData.csv');

    RRTData = table(meanD(:,4), stdD(:,4), meanCurv(:,4), stdCurv(:,4),...
        meanIAdH(:,4), stdIAdH(:,4), meanISdH(:,4), stdISdH(:,4), ...
        meanD(:,8), stdD(:,8), meanCurv(:,8), stdCurv(:,8),...
        meanIAdH(:,8), stdIAdH(:,8), meanISdH(:,8), stdISdH(:,8), mazeNr);
    RRTData.Properties.VariableNames(1:17) = {'meanD_o', 'stdD_o', 'meanCurv_o',...
        'stdCurv_o', 'meanIAdH_o', 'stdIAdH_o', 'meanISdH_o', 'stdISdH_o',...
        'meanD_no','stdD_no', 'meanCurv_no', 'stdCurv_no', 'meanIAdH_no',...
        'stdIAdH_no', 'meanISdH_no', 'stdISdH_no', 'Maze'};
    writetable(RRTData,'data/RRTData.csv');
end



