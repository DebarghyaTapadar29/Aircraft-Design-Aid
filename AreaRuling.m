% MATLAB Area Rule Analysis Tool (Optimized)



% 1. Load STL File
[filename, pathname] = uigetfile('F24.stl', 'Select STL File');
if isequal(filename,0) || isequal(pathname,0)
   disp('User selected Cancel')
   return;
else
   disp(['User selected ', fullfile(pathname, filename)])
end

try
    stlData = stlread(fullfile(pathname, filename));
catch
    error('Error reading STL file. Ensure it is a valid binary STL file.');
end

% 2. Define Parameters
numSlices = 100; % Number of slices along the aircraft length (now X-axis)
smoothingWindowSize = 5; % Window size for moving average smoothing
idealAreaFactor = 0.8; % Scale factor for the ideal Sears-Haack area distribution (adjust as needed)
slicePlaneResolution = 20; % Reduced resolution for slice plane meshgrid (important for speed)
% Reduced resolution for triangle iteration. A higher number means fewer
% triangles are checked, improving speed but potentially reducing accuracy
triangleIterationSkip = 5;

% 3. Determine Aircraft Orientation and Length
vertices = stlData.Points;
minX = min(vertices(:,1));
maxX = max(vertices(:,1));
minY = min(vertices(:,2));
maxY = max(vertices(:,2));
minZ = min(vertices(:,3));
maxZ = max(vertices(:,3));

% --- CHANGE: Assuming the longest dimension (length) is now along X-axis ---
aircraftLength = maxX - minX;
% Calculate center based on bounds of the model in all 3 dimensions
aircraftCenter = [(minX + maxX)/2, (minY + maxY)/2, (minZ + maxZ)/2];

% 4. Slice and Calculate Cross-Sectional Areas (Optimized)
% --- CHANGE: Slices are now along the X-axis ---
sliceLocations = linspace(minX, maxX, numSlices);
actualAreas = zeros(1, numSlices);

% Pre-allocate matrices for faster calculations
numTriangles = size(stlData.ConnectivityList, 1);
distances = zeros(3,1); % Pre-allocate for distances
intersections = zeros(2,3); % Pre-allocate for max 2 intersection points

% Initialize waitbar
% --- CHANGE: Update waitbar description ---
hWaitbar = waitbar(0, 'Calculating Slices along X-axis...', 'Name', 'Area Rule Analysis');
startTime = tic; % Start timer

% Pre-calculate min/max Y/Z for slice planes (perpendicular to X)
ymin = minY;
ymax = maxY;
zmin = minZ;
zmax = maxZ;


for i = 1:numSlices
    % --- CHANGE: Slice plane is now at a constant X value ---
    slicePlaneX = sliceLocations(i);

    % Create slice plane (normal to X-axis) - using Y and Z for meshgrid
    % --- CHANGE: Meshgrid uses Y and Z for the plane perpendicular to X ---
    [Y_plane, Z_plane] = meshgrid(ymin: (ymax-ymin)/slicePlaneResolution:ymax, zmin:(zmax-zmin)/slicePlaneResolution:zmax);
    % --- CHANGE: X coordinate is constant for this plane ---
    X_plane = slicePlaneX * ones(size(Y_plane)); % Use size of Y_plane/Z_plane

    % Visualization removed from the loop for performance. Add back if needed for debugging.
    % hSlice = surf(X_plane, Y_plane, Z_plane, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.2); % Transparent slice plane
    % sliceHandles = [sliceHandles, hSlice]; % Store slice plane handles

    % Calculate intersection with triangles (simplified)
    totalArea = 0;

    % Iterate over a subset of triangles (speed improvement - can adjust triangleIterationSkip)
    for j = 1:triangleIterationSkip:numTriangles
        triangleVertices = vertices(stlData.ConnectivityList(j,:), :);

        % Check if triangle intersects the plane (vectorized)
        % --- CHANGE: Check distance from X-plane using X-coordinates ---
        distances = triangleVertices(:,1) - slicePlaneX; % Check distance from X-plane
        if sign(distances(1)) ~= sign(distances(2)) || ...
           sign(distances(1)) ~= sign(distances(3)) || ...
           sign(distances(2)) ~= sign(distances(3))

           % Simplified intersection calculation (can be improved for accuracy)
           % (Approximating intersected area as area of triangle projected on the slice plane)

           % Find intersection points (approximate) - can be improved with more robust method
           numIntersections = 0;
           for k = 1:3
               nextK = mod(k,3) + 1;
               % --- CHANGE: Interpolate based on distance along X-axis ---
               if (distances(k)*distances(nextK) <=0) % Intersects
                 t = (slicePlaneX - triangleVertices(k,1))/(triangleVertices(nextK,1) - triangleVertices(k,1)); % Interpolate based on X
                 intersectionPoint = triangleVertices(k,:) + t*(triangleVertices(nextK,:) - triangleVertices(k,:));
                 numIntersections = numIntersections + 1;
                 if numIntersections <= 2 % Store only up to 2 points (simplified approach)
                     intersections(numIntersections,:) = intersectionPoint;
                 end
               end
           end

           % If 2 intersection points are found, approximate the cross-sectional area
           % Note: A more accurate method would involve finding ALL intersection
           % points for the slice plane and forming a polygon whose area is calculated.
           % This simplified method approximates area from a single intersecting edge pair.
           if numIntersections >= 2
                % Approximating area using a line length scaled by average projected "height"
                % --- CHANGE: Line length is in the YZ plane (perpendicular to X) ---
               lineLength = norm(intersections(1,2:3) - intersections(2,2:3)); % Norm in YZ plane
                % --- CHANGE: Height scale is average distance from centroid in YZ plane ---
               heightScale = mean(sqrt(sum((triangleVertices(:,2:3) - mean(triangleVertices(:,2:3))).^2, 2))); % Average distance from centroid in YZ
               totalArea = totalArea + lineLength*heightScale; % Approximation
           end
        end
    end

    actualAreas(i) = totalArea;
    % Visualization removed from the loop
    % delete(sliceHandles); % Remove old slices for next iteration

    % Update waitbar and estimate time remaining
    progress = i / numSlices;
    elapsedTime = toc(startTime);
    estimatedTotalTime = elapsedTime / progress;
    estimatedTimeRemaining = estimatedTotalTime - elapsedTime;

    % --- CHANGE: Update waitbar description ---
    waitbar(progress, hWaitbar, sprintf('Calculating Slices along X-axis... (%.1f%%) Time Remaining: %.1f s', progress * 100, estimatedTimeRemaining));
end

% Visualization outside the loop
figure; hold on; % Setup for 3D visualization
trisurf(stlData, 'FaceColor', 'cyan', 'EdgeColor', 'none'); % Display STL model
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('STL Model');
view(3); % Ensure a 3D view
hold off;


% Close waitbar
close(hWaitbar);


% 5. Smoothing (Moving Average)
actualAreasSmoothed = movmean(actualAreas, smoothingWindowSize);

% 6. Calculate Ideal Sears-Haack Area Distribution
idealAreas = zeros(1, numSlices);
for i = 1:numSlices
    % --- CHANGE: Axial position is along the X-axis relative to the center X ---
    axialPosition = sliceLocations(i) - aircraftCenter(1); % Use center X coordinate

    % Sears-Haack radius formula based on axial position
    r = sqrt(1 - (2*axialPosition/aircraftLength)^2);
    if ~isreal(r) || isnan(r)
      r = 0; % Handle out of bounds or complex numbers near ends
    end

    % Ideal area (Area of a circle with the calculated radius)
    idealArea = pi * (aircraftLength/2 * r)^2;
    idealAreas(i) = idealArea;
end

% Scale the ideal area distribution
maxActualArea = max(actualAreasSmoothed);
idealAreasScaled = idealAreas * (idealAreaFactor * maxActualArea / max(idealAreas));


% 7. Graphical Comparison
figure;
% --- CHANGE: Plotting against sliceLocations (which are now X values) ---
plot(sliceLocations, actualAreas, 'b-', 'DisplayName', 'Actual Area (Unsmoothed)');
hold on;
plot(sliceLocations, actualAreasSmoothed, 'r-', 'LineWidth', 2, 'DisplayName', 'Actual Area (Smoothed)');
plot(sliceLocations, idealAreasScaled, 'g-', 'LineWidth', 2, 'DisplayName', 'Ideal Sears-Haack Area');
hold off;
% --- CHANGE: Update X-axis label to indicate X-axis ---
xlabel('Axial Position (X)');
ylabel('Cross-Sectional Area');
title('Area Rule Analysis (X-axis)');
legend('Location', 'best');
grid on;


% 8. Area Deviation Metric (Optional)
areaDeviation = sum(abs(actualAreasSmoothed - idealAreasScaled)); % Sum of absolute differences
disp(['Area Deviation Metric: ', num2str(areaDeviation)]);

disp('Analysis Complete.');
