% % image4
% worldPos = [[-330, -175]; [-405, -450]; [-800, -50]; [-875, -318]];
% % image5
% worldPos = [[-425, 0]; [-335, -270]; [-880, -160]; [-790, -425]];
clear all;
global zHeight;
global home;
host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
% host = '192.168.230.128'; % THIS IP ADDRESS MUST BE USED FOR THE VMWARE
%host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
portRTDE = 30003;
portVac = 63352;
rtde = rtde(host, portRTDE);
rtde.timeout = 120;
%vacuum = vacuum(host, portVac);
startup_rvc; % Startup the rvc toolbox
blend = 0;
v = 0.2;
a = 0.5;
zHeight = 20; %should be 0
home = [-588.53, -133.30, 227, 2.221, 2.221, 0.00, a, v, 0, blend];
zHeight = 20;

[markerCentres, tetrisImg] = getMarkerPos();

% Table markers
% table marker coordinates real
tablePos = [[-230, 60]; [-230, -520]; [-990, 60]; [-990, -520]];
tableTrans = fitgeotrans(tablePos, markerCentres(1:4, 1:2), 'projective');

%board markers
boardPos = transformPointsInverse(tableTrans, markerCentres(5:8, 1:2));
fprintf("id %d: %f, %f\n", 5, boardPos(1, 1), boardPos(1, 2));
fprintf("id %d: %f, %f\n", 6, boardPos(2, 1), boardPos(2, 2));
fprintf("id %d: %f, %f\n", 7, boardPos(3, 1), boardPos(3, 2));
fprintf("id %d: %f, %f\n", 8, boardPos(4, 1), boardPos(4, 2));

% the coodinates to re-organise the image into a 2D format
windowPos = [[50, 50]; [340, 50]; [50, 540]; [340, 540]];

% This is the image position of the desired markers. the indices are chosen
% such that we only select the border markers not the outer ones
imgPos = markerCentres(5:8, 1:2);
img2windowTrans = fitgeotrans(imgPos, windowPos, 'projective');
windowDimensions = [590 390 3];
% the imref2d must specify the size of the window
tetrisImg = imwarp(tetrisImg, img2windowTrans, OutputView=imref2d(windowDimensions));

blueMask = (tetrisImg(:,:,1) < 150)&(tetrisImg(:,:,2) < 100) & (tetrisImg(:, :, 3) > 130);
blueMask = bwareaopen(blueMask, 100);
%figure(2)
%imshow(blueMask)

redMask = (tetrisImg(:,:,1) > 135)&(tetrisImg(:,:,2) < 130) & (tetrisImg(:, :, 3) < 130);
redMask = bwareaopen(redMask, 100);
%figure(3)
%imshow(redMask)

greenMask = (tetrisImg(:, :, 1) < 115)&(tetrisImg(:, :, 2) >= 120)&(tetrisImg(:, :, 3) < 130);
greenMask = bwareaopen(greenMask, 100);
%figure(4)
%imshow(greenMask)

s = regionprops(blueMask, 'centroid');
blueCentroids = cat(1, s.Centroid);

s = regionprops(redMask, 'centroid');
redCentroids = cat(1, s.Centroid);

s = regionprops(greenMask, 'centroid');
greenCentroids = cat(1, s.Centroid);

% puts text at the centre of the box with the value of the centre
% position
tetrisImg = labelImg(tetrisImg, blueCentroids, "blue");
tetrisImg = labelImg(tetrisImg, redCentroids, "red");
tetrisImg = labelImg(tetrisImg, greenCentroids, "green");
figure(1)
imshow(tetrisImg)

window2realTrans = fitgeotrans(windowPos, boardPos, 'projective');

blueReal = transformPointsForward(window2realTrans, blueCentroids);
redReal = transformPointsForward(window2realTrans, redCentroids);
greenReal = transformPointsForward(window2realTrans, greenCentroids);

desiredGrid = [1 1 1 1 1 1 1 1 1 1
               1 1 0 0 0 0 1 1 1 1
               1 0 1 4 1 0 0 2 0 1
               1 2 0 1 2 0 1 0 0 1
               1 2 0 2 1 0 2 0 3 1
               1 1 0 0 0 0 0 0 1 1
               1 1 1 1 1 1 1 1 1 1];
desiredGrid = flipud(desiredGrid);
[destY, destX] = find(desiredGrid==4);
[startY, startX] = find(desiredGrid==3);
dest = [destX, destY];
start = [startX, startY];

% part C
% rtde.movej(home(1:6), 'pose', a, v, 0, blend);
% realGrid = modifyRealGrid(desiredGrid, start, dest, windowPos, blueCentroids, boardPos, rtde);
% rtde.movej(home(1:6), 'pose', a, v, 0, blend);

windowPath = getPath(windowPos);
tetrisImg = insertShape(tetrisImg,"line", windowPath,Opacity=1,ShapeColor="green",LineWidth=4);
figure(2)
imshow(tetrisImg)

realPath = transformPointsForward(window2realTrans, windowPath);

plotRealBoard(tablePos, boardPos, blueReal, redReal, greenReal, realPath);
path = createPath(realPath);
poses = rtde.movej(path(1, 1:6), 'pose', a, v, 0, blend);
poses = cat(1, poses, rtde.movej(path(2, 1:6), 'pose', a, v, 0, blend));
% vacuum.grip();
% pause(5);
poses = cat(1, poses, rtde.movej(path(3:numrows(path) - 1, :)));
rtde.drawPath(poses);
% vacuum.release();
% pause(5);
poses = cat(1, poses, rtde.movej(path(numrows(path), 1:6), 'pose', a, v, 0, blend));
rtde.close();
%vacuum.close();

function currentGrid = modifyRealGrid(desiredGrid, start, dest, windowPos, blueCentroids, boardPos, rtde)
    global home
    global zHeight
    gridPos = [[9, 6]; [9, 2]; [2, 6]; [2, 2]];
    window2grid = fitgeotrans(windowPos, gridPos, 'projective');
    currentGrid = zeros(7, 10);
    redGrid = find(desiredGrid == 1);
    currentGrid(redGrid) = 1;
    currentGrid(start(2), start(1)) = 3;
    currentGrid(dest(2), dest(1)) = 4;
    blueList = [];

    grid2real = fitgeotrans(gridPos, boardPos, 'projective');
    for i = 1:numrows(blueCentroids)
        blueGrid = round(transformPointsForward(window2grid, blueCentroids(i, :)));
        currentGrid(blueGrid(2), blueGrid(1)) = 2;
        blueList = cat(1, blueList, blueGrid);
    end
    
    poses = [];
    camlist = webcamlist;
    cam = webcam(2);
    while ~isequal(desiredGrid, currentGrid)
        for row = 1:numrows(desiredGrid)
            for col = 1:numcols(desiredGrid)
                if desiredGrid(row, col) == 2 && currentGrid(row, col) ~= 2
                    for i = 1:numrows(blueList)
                        if desiredGrid(blueList(i, 2), blueList(i, 1)) ~= 2
                            currentGrid(row, col) = 2;
                            currentGrid(blueList(i, 2), blueList(i, 1)) = 0;
                            oldPos = transformPointsForward(grid2real, [blueList(i, 2), blueList(i, 1)]);
                            newPos = transformPointsForward(grid2real, [col, row]);
                            poses = cat(1, poses, rtde.movej([oldPos, zHeight, home(4:6)], 'pose', home(7:10)));
                            %vacuum.grip();
                            %pause(5);
                            poses = cat(1, poses, rtde.movej([oldPos, zHeight + 20, home(4:6)], 'pose', home(7:10)));
                            poses = cat(1, poses, rtde.movej([newPos, zHeight, home(4:6)], 'pose', home(7:10)));
                            %vacuum.release();
                            %pause(5);
                            poses = cat(1, poses, rtde.movej([newPos, zHeight + 50, home(4:6)], 'pose', home(7:10)));
                            blueList(i, :) = [col, row];
                            break
                        end
                    end
                end
            end
        end
       tetrisImg = snapshot(cam);
       blueMask = (tetrisImg(:,:,1) < 150)&(tetrisImg(:,:,2) < 100) & (tetrisImg(:, :, 3) > 130);
       blueMask = bwareaopen(blueMask, 100);
       s = regionprops(blueMask, 'centroid');
       blueCentroids = cat(1, s.Centroid);
       blueList = [];
       for i = 1:numrows(blueCentroids)
        blueGrid = round(transformPointsForward(window2grid, blueCentroids(i, :)));
        currentGrid(blueGrid(2), blueGrid(1)) = 2;
        blueList = cat(1, blueList, blueGrid);
       end
    end
    figure(4)
    rtde.drawPath(poses);
end

function [markerCentres, img] = getMarkerPos()
    % camlist = webcamlist;
    % cam = webcam(2);
    % img = snapshot(cam);
    img = imread('3.jpg');
    [ids, locs, detectedFamily] = readArucoMarker(img, "DICT_4X4_250");

    numMarkers = length(ids);
    markerCentres = zeros([numMarkers 3]);
    for i = 1:numMarkers
        % location of the 4 corners for the current marker
        currentLoc = locs(:, :, i);

        % obtains the centre of the marker
        markerCentres(i, :) = [mean(currentLoc), ids(i)];
    end
    markerCentres = sortrows(markerCentres, 3);
end

function newImg = labelImg(img, centroids, colour)
    newImg = img;
    for i=1:numrows(centroids)
        newImg = insertText(newImg, centroids(i, :), colour, FontSize=30, BoxOpacity=1);
    end
end

function windowPath = getPath(windowPos)
       % 8 rows (wide) and 10 columns (tall)
    grid = [1 1 1 1 1 1 1 1 1 1
            1 1 0 0 0 0 0 0 1 1
            1 0 0 0 1 0 0 0 0 1
            1 0 0 1 0 0 1 0 0 1
            1 4 1 0 0 1 1 0 3 1
            1 1 0 0 0 0 0 0 1 1
            1 1 1 1 1 1 1 1 1 1];
    grid = flipud(grid);
    scalingFactor = 50;
    [destY, destX] = find(grid==4);
    [startY, startX] = find(grid==3);
    grid(startY, startX) = 0;
    grid(destY, destX) = 0;
    [freeY, freeX] = find(grid==0);
    
    scalingMatrix = ones(scalingFactor);
    grid = kron(grid, scalingMatrix);
    
    cornerPos = getCornerPos(grid);
    grid = fillInGrid(cornerPos, grid);
    
    start = scaledPos(startX, startY, scalingFactor);
    dest = scaledPos(destX, destY, scalingFactor);
    freeX = freeX * scalingFactor - scalingFactor / 2;
    freeY = freeY * scalingFactor - scalingFactor / 2;
    free = [freeX, freeY];
    
    tL = [9 * scalingFactor - scalingFactor / 2; 6 * scalingFactor - scalingFactor / 2];
    tR = [9 * scalingFactor - scalingFactor / 2; 2 * scalingFactor - scalingFactor / 2];
    bL = [2 * scalingFactor - scalingFactor / 2; 6 * scalingFactor - scalingFactor / 2];
    bR = [2 * scalingFactor - scalingFactor / 2; 2 * scalingFactor - scalingFactor / 2];
    
    bug = Bug2(grid);
    algPath = bug.query(start, dest);
    
    mapPath = getMapPath(algPath, free);
    mapPath = removeDuplicates(mapPath);
    
    mapPos = [tL'; tR'; bL'; bR'];
    map2window = fitgeotrans(mapPos, windowPos, 'projective');
    windowPath = transformPointsForward(map2window, mapPath);
end

function pos = getCornerPos(grid)
    pos = [];
    for row = 2:numrows(grid) - 1
        for col = 2 :numcols(grid) - 1
            current = grid(row, col);
            left = grid(row, col - 1);
            right = grid(row, col + 1);
            bot = grid(row + 1, col);
            top = grid(row - 1, col);
            % 0 0 0 0
            % 0 1 1c 0
            % 0 1 1 0
            % 0 0 0 0
            if (current == 1 && right == 0 && top == 0 && left == 1 && bot == 1)
                pos = cat(1, pos, [row - 1, col]);
                pos = cat(1, pos, [row - 1, col + 1]);
                pos = cat(1, pos, [row, col + 1]);
            elseif (current == 1 && left == 0 && top == 0 && right == 1 && bot == 1)
                pos = cat(1, pos, [row - 1, col]);
                pos = cat(1, pos, [row - 1, col - 1]);
                pos = cat(1, pos, [row, col - 1]);
            elseif (current == 1 && left == 0 && bot == 0 && right == 1 && top == 1)
                pos = cat(1, pos, [row + 1, col]);
                pos = cat(1, pos, [row, col - 1]);
                pos = cat(1, pos, [row + 1, col - 1]);
            elseif (current == 1 && right == 0 && bot == 0 && left == 1 && top == 1)
                pos = cat(1, pos, [row + 1, col]);
                pos = cat(1, pos, [row + 1, col + 1]);
                pos = cat(1, pos, [row, col + 1]);
            end
        end
    end
end

function grid = fillInGrid(pos, grid)
    for i = 1:numrows(pos)
        grid(pos(i, 1), pos(i, 2)) = 1;
    end
end

function pos = scaledPos(x, y, scalingFactor)
   pos = [1, 1];
   pos(1) = x * scalingFactor - scalingFactor / 2;
   pos(2) = y * scalingFactor - scalingFactor / 2;
end

function mapPath = getMapPath(algPath, free)
    mapPath = [];
    for i = 1:numrows(algPath)
       shortestDist = abs(norm(free(1, :) - algPath(i, :)));
       freePoint = free(1, :);
       for j = 2: numrows(free)
            dist = abs(norm(free(j, :) - algPath(i, :)));
            if dist < shortestDist
                shortestDist = dist;
                freePoint = free(j, :);
            end
       end
       mapPath = cat(1, mapPath, freePoint);
    end
end

function path = removeDuplicates(path)
    diffRows = diff(path, 1);
    rowsToDelete = find(all(diffRows == 0, 2)) + 1;
    path(rowsToDelete, :) = [];
end

function plotRealBoard(tablePos, boardPos, blueReal, redReal, greenReal, realPath)
    figure(3)
    hold on
    plot(tablePos(:, 1), tablePos(:, 2), '.');
    plot(boardPos(:, 1), boardPos(:, 2), '.');
    plot(blueReal(:, 1), blueReal(:, 2), '*');
    plot(redReal(:, 1), redReal(:,2), '^');
    plot(greenReal(:, 1), greenReal(:,2), '.');
    plot(realPath(:, 1), realPath(:, 2))
    hold off
end

function path = createPath(realPath)
    global zHeight
    global home
    path = home;
    path = cat(1, path, [realPath(1, :), zHeight, home(4:10)]);
    for i = 2:numrows(realPath)
        path = cat(1, path, [realPath(i, :), zHeight + 5, home(4:10)]);
    end
    path = cat(1, path, home);
end